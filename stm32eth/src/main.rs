#![no_std]
#![no_main]

use panic_halt as _;

#[rtic::app(device = stm32f1xx_hal::pac, dispatchers = [ADC, EXTI0])]
mod app {
    use cortex_m::singleton;
    use embedded_hal::spi::{Mode, Phase, Polarity}; // The runtime

    use stm32f1xx_hal::{
        pac::USART3,
        prelude::*,
        serial::{Config, Serial, Tx},
        spi::{NoMiso, Spi, SpiBitFormat},
        gpio::{Output, PA5, PA7, PB13, PB15, PC6}, device::SPI2,
        dma::dma1,
        timer::{Event, Counter},
        pac
    };

    use core::fmt::Write;
    use core::arch::asm;

    pub const MODE: Mode = Mode {
        phase: Phase::CaptureOnFirstTransition,
        polarity: Polarity::IdleLow,
    };

    #[shared]
    struct Shared {
        tx_pins: (PA5<Output>, PA7<Output>),
        uart_tx: Tx<USART3>
    }

    type SpiPeriph = (SPI2, PB13, PB15, dma1::C4);

    #[local]
    struct Local {
        rx_spi_periph: Option<SpiPeriph>,
        rx_buf: Option<&'static mut [u8; 2048]>,
        rx_pin_cs: PC6,
        nlp_timer: Counter<pac::TIM1, 1000>,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        let dp = cx.device;

        let rcc = dp.RCC.constrain();
        let mut gpioc = dp.GPIOC.split();

        let mut afio = dp.AFIO.constrain();
        let mut gpiob = dp.GPIOB.split();
        let mut gpioa = dp.GPIOA.split();

        let mut flash = dp.FLASH.constrain();
        let clocks = rcc
            .cfgr
            .use_hse(8.MHz())
            .sysclk(40.MHz())
            .pclk1(20.MHz())
            .freeze(&mut flash.acr);

        let pin_cs = gpioc.pc6.into_floating_input(&mut gpioc.crl);

        let pin_tx = gpiob.pb10.into_alternate_push_pull(&mut gpiob.crh);
        let pin_rx = gpiob.pb11;
        let serial = Serial::new(
            dp.USART3,
            (pin_tx, pin_rx),
            &mut afio.mapr,
            Config::default().baudrate(115200.bps()),
            &clocks,
        );

        let (uart_tx, _rx) = serial.split();

        // SPI2
        let pin_sck = gpiob.pb13;
        let pin_mosi = gpiob.pb15;

        let dma = dp.DMA1.split();

        let buf = Some(singleton!(: [u8; 2048] = [0; 2048]).unwrap());
        let periph = Some((dp.SPI2, pin_sck, pin_mosi, dma.4));
        let mut pin_tx_sck = gpioa.pa5.into_push_pull_output(&mut gpioa.crl);
        let mut pin_tx_mosi = gpioa.pa7.into_push_pull_output(&mut gpioa.crl);

        pin_tx_sck.set_high();
        pin_tx_mosi.set_low();

        task_rx::spawn().unwrap();

        let mut timer = dp.TIM1.counter_ms(&clocks);
        timer.start(16.millis()).unwrap();
        timer.listen(Event::Update);

        (
            Shared {
                tx_pins: (pin_tx_sck, pin_tx_mosi),
                uart_tx: uart_tx,
            },
            Local {
                rx_spi_periph: periph,
                rx_buf: buf,
                rx_pin_cs: pin_cs,
                nlp_timer: timer,
            },
            init::Monotonics()
        )
    }

    #[task(local = [rx_spi_periph, rx_buf, rx_pin_cs], shared = [uart_tx])]
    fn task_rx(mut ctx: task_rx::Context) {
        let periph = ctx.local.rx_spi_periph;
        let buf = ctx.local.rx_buf;
        let pin_cs = ctx.local.rx_pin_cs;
        loop {
            // init SPI and start transfer regardless of CS
            let (spi, sck, mosi, dma) = periph.take().unwrap();
            let mut spi2 = Spi::spi2_slave(spi, (sck, NoMiso, mosi), MODE);
            spi2.bit_format(SpiBitFormat::LsbFirst);
            let spi_dma_rx = spi2.with_rx_dma(dma);

            let transfer = {
                let buf = buf.take().unwrap();
                spi_dma_rx.read(buf)
            };
            // wait for CS to go high
            while !pin_cs.is_high() {}
            // wait for CS to go low
            while !pin_cs.is_low() {}

            // finish transfer
            {
                let (ret_buf, ret_rx) = transfer.stop();
                let ndtr = ret_rx.channel.get_ndtr() as usize;
                if ret_buf.len() - ndtr != 0 {
                    ctx.shared.uart_tx.lock(|uart_tx| {
                        for i in 0..ret_buf.len() - ndtr {
                            write!(uart_tx, "{:02X} ", ret_buf[i]).ok();
                        }
                        uart_tx.write_char('\n').ok();
                    });
                }
                buf.replace(ret_buf);

                // release SPI to reset it
                let (spi, dma) = ret_rx.release();
                let (spi, (sck, _, mosi)) = spi.release();
                periph.replace((spi, sck, mosi, dma));
            }
        }
    }

    #[task(binds = TIM1_UP_TIM16, local = [nlp_timer], shared = [uart_tx, tx_pins], priority = 2)]
    fn task_nlp(mut ctx: task_nlp::Context) {
        ctx.shared.tx_pins.lock(|(_pin_sck, pin_mosi)| {
            pin_mosi.set_high();
            unsafe {
                asm!(
                    "nop",
                    "nop");
            }
            pin_mosi.set_low();
        });
        ctx.local.nlp_timer.clear_interrupt(Event::Update);
    }
}

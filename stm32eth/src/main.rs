#![no_std]
#![no_main]

mod tx_frame_buf;

use panic_halt as _;

#[rtic::app(device = stm32f1xx_hal::pac, dispatchers = [ADC])]
mod app {
    use cortex_m::singleton;
    use embedded_hal::{
        spi::{Mode, Phase, Polarity},
    }; // The runtime

    use stm32f1xx_hal::{
        afio::MAPR,
        device::SPI1,
        device::SPI2,
        dma::dma1,
        gpio::{Cr, Edge, ExtiPin, Output, PA0, PA5, PA7, PB13, PB15, PC6},
        pac,
        pac::USART3,
        prelude::*,
        rcc::Clocks,
        serial::{Config, Serial, Tx},
        spi::{NoMiso, Spi, SpiBitFormat},
        timer::{Counter, Event},
    };

    use core::arch::asm;
    use core::fmt::Write;

    use crate::tx_frame_buf::TxFrameBuf;

    pub const MODE: Mode = Mode {
        phase: Phase::CaptureOnFirstTransition,
        polarity: Polarity::IdleLow,
    };

    type SpiRxPeriph = (SPI2, PB13, PB15, dma1::C4);
    pub struct SpiTxPeriph {
        spi: SPI1,
        sck: PA5<Output>,
        mosi: PA7<Output>,
        dma: dma1::C3,
        cr: Cr<'A', false>,
        mapr: MAPR,
        clocks: Clocks,
    }

    #[shared]
    struct Shared {
        tx_spi_periph: Option<SpiTxPeriph>,
        uart_tx: Tx<USART3>,
    }

    #[local]
    struct Local {
        rx_spi_periph: Option<SpiRxPeriph>,
        rx_pin_cs: PC6,
        nlp_timer: Counter<pac::TIM1, 1000>,
        button_pin: PA0,
        tx_buf: Option<&'static mut [u8; 512]>,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        let mut dp = cx.device;

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

        let periph = Some((dp.SPI2, pin_sck, pin_mosi, dma.4));
        let mut pin_tx_sck = gpioa.pa5.into_push_pull_output(&mut gpioa.crl);
        let mut pin_tx_mosi = gpioa.pa7.into_push_pull_output(&mut gpioa.crl);

        pin_tx_sck.set_low();
        pin_tx_mosi.set_low();

        let mut timer = dp.TIM1.counter_ms(&clocks);
        timer.start(16.millis()).unwrap();
        timer.listen(Event::Update);

        let mut button_pin = gpioa.pa0.into_floating_input(&mut gpioa.crl);
        button_pin.make_interrupt_source(&mut afio);
        button_pin.enable_interrupt(&mut dp.EXTI);
        button_pin.trigger_on_edge(&mut dp.EXTI, Edge::Rising);

        task_rx::spawn().unwrap();

        (
            Shared {
                tx_spi_periph: Some(SpiTxPeriph {
                    spi: dp.SPI1,
                    sck: pin_tx_sck,
                    mosi: pin_tx_mosi,
                    dma: dma.3,
                    cr: gpioa.crl,
                    mapr: afio.mapr,
                    clocks: clocks,
                }),
                uart_tx: uart_tx,
            },
            Local {
                rx_spi_periph: periph,
                rx_pin_cs: pin_cs,
                nlp_timer: timer,
                button_pin: button_pin,
                tx_buf: singleton!(: [u8; 512] = [0; 512]),
            },
            init::Monotonics(),
        )
    }

    #[task(local = [rx_spi_periph, rx_pin_cs], shared = [uart_tx])]
    fn task_rx(mut ctx: task_rx::Context) {
        let periph = ctx.local.rx_spi_periph;
        let mut buf = Some(singleton!(: [u8; 2048] = [0; 2048]).unwrap());
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

    #[task(binds = TIM1_UP_TIM16, local = [nlp_timer], shared = [uart_tx, tx_spi_periph], priority = 2)]
    fn task_nlp(mut ctx: task_nlp::Context) {
        ctx.shared.tx_spi_periph.lock(|periph| {
            if let Some(SpiTxPeriph { mosi: pin_mosi, .. }) = periph {
                pin_mosi.set_high();
                unsafe {
                    asm!("nop", "nop");
                }
                pin_mosi.set_low();
            }
        });
        ctx.local.nlp_timer.clear_interrupt(Event::Update);
    }

    #[task(binds = EXTI0, priority = 3, local = [button_pin, tx_buf], shared = [tx_spi_periph, uart_tx])]
    fn task_btn(mut ctx: task_btn::Context) {
        ctx.local.button_pin.clear_interrupt_pending_bit();

        ctx.shared.uart_tx.lock(|uart_tx| {
            writeln!(uart_tx, "Button is pressed!").ok();
        });
        ctx.shared.tx_spi_periph.lock(|periph| {
            let buf = ctx.local.tx_buf.take().unwrap();
            let mut p = periph.take().unwrap();
            let sck_alt = p.sck.into_alternate_push_pull(&mut p.cr);
            let mosi_alt = p.mosi.into_alternate_push_pull(&mut p.cr);
            let mut spi1 = Spi::spi1(
                p.spi,
                (sck_alt, NoMiso, mosi_alt),
                &mut p.mapr,
                Mode {
                    polarity: Polarity::IdleHigh,
                    phase: Phase::CaptureOnFirstTransition,
                },
                10.MHz(),
                p.clocks,
            );
            spi1.bit_format(SpiBitFormat::LsbFirst);

            let frame = [
                0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
                0x11, 0x22, 0x33, 0x44, 0x55, 0x66,
                0x90, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05,
                0x06, 0x07, 0x08, 0x09, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x21,
                0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36,
                0x37, 0x38, 0x39,
            ];
            let var_len_buf = TxFrameBuf::new(buf, &frame);

            let spi_dma = spi1.with_tx_dma(p.dma);

            let transfer = spi_dma.write(var_len_buf);

            let (var_len_buf, spi_dma) = transfer.wait();
            let buf = var_len_buf.release();

            let (spi1, dma) = spi_dma.release();
            p.dma = dma;

            while spi1.is_busy() {
            }

            let (spi, (sck_alt, _, mosi_alt)) = spi1.release();
            p.spi = spi;
            p.sck = sck_alt.into_push_pull_output(&mut p.cr);
            p.mosi = mosi_alt.into_push_pull_output(&mut p.cr);
            p.sck.set_low();
            p.mosi.set_low();
            periph.replace(p);
            ctx.local.tx_buf.replace(buf);
        });
    }
}

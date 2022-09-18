#![no_std]
#![no_main]

mod device;
mod receiver;
mod transmitter;
mod tx_frame_buf;

use panic_halt as _;

#[rtic::app(device = stm32f1xx_hal::pac, dispatchers = [ADC])]
mod app {
    use cortex_m::singleton;

    use stm32f1xx_hal::{
        gpio::{Edge, ExtiPin, PA0},
        pac::USART3,
        prelude::*,
        serial::{Config, Serial, Tx},
    };

    use core::fmt::Write;

    use crate::{receiver::Receiver, transmitter::Transmitter};

    #[shared]
    struct Shared {
        uart_tx: Tx<USART3>,
        receiver: Receiver,
    }

    #[local]
    struct Local {
        button_pin: PA0,
        transmitter: Transmitter,
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
        let mut pin_cs = gpioc.pc6.into_floating_input(&mut gpioc.crl);
        pin_cs.make_interrupt_source(&mut afio);
        pin_cs.enable_interrupt(&mut dp.EXTI);
        pin_cs.trigger_on_edge(&mut dp.EXTI, Edge::Falling);

        let dma = dp.DMA1.split();

        let rx_buf =
            singleton!(: [u8; crate::receiver::BUFFER_LEN] = [0; crate::receiver::BUFFER_LEN])
                .unwrap();
        let receiver = Receiver::new(dp.SPI2, pin_sck, pin_mosi, pin_cs, dma.4, rx_buf);

        let pin_tx_sck = gpioa.pa5.into_push_pull_output(&mut gpioa.crl);
        let pin_tx_mosi = gpioa.pa7.into_push_pull_output(&mut gpioa.crl);
        let pin_nlp_disa = gpiob.pb1.into_push_pull_output(&mut gpiob.crl);

        let tx_buf = singleton!(: [u8; crate::transmitter::BUFFER_LEN] = [0; crate::transmitter::BUFFER_LEN]).unwrap();

        let mut button_pin = gpioa.pa0.into_floating_input(&mut gpioa.crl);
        button_pin.make_interrupt_source(&mut afio);
        button_pin.enable_interrupt(&mut dp.EXTI);
        button_pin.trigger_on_edge(&mut dp.EXTI, Edge::Rising);

        let transmitter = Transmitter::new(
            dp.SPI1,
            pin_tx_sck,
            pin_tx_mosi,
            pin_nlp_disa,
            dma.3,
            gpioa.crl,
            afio.mapr,
            clocks,
            tx_buf,
        );

        (
            Shared {
                uart_tx: uart_tx,
                receiver,
            },
            Local {
                button_pin: button_pin,
                transmitter,
            },
            init::Monotonics(),
        )
    }

    #[idle(shared = [receiver, uart_tx])]
    fn idle(mut ctx: idle::Context) -> ! {
        ctx.shared.receiver.lock(|receiver| {
            receiver.start_listen();
        });
        loop {
            let d = ctx.shared.receiver.lock(|receiver| receiver.try_get_data());
            if let Some((buf, len)) = d {
                ctx.shared.uart_tx.lock(|tx| {
                    for i in 0..len {
                        write!(tx, "{:02X} ", buf[i]).ok();
                    }
                    tx.write_char('\n').ok();
                });
                ctx.shared.receiver.lock(|receiver| {
                    receiver.return_buffer(buf);
                });
            }
        }
    }

    #[task(binds = EXTI9_5, priority = 3, shared = [receiver, uart_tx])]
    fn task_cs_down(mut ctx: task_cs_down::Context) {
        ctx.shared.uart_tx.lock(|tx| {
            writeln!(tx, "cs down").ok();
            ctx.shared.receiver.lock(|receiver| {
                let result = receiver.on_frame_end();
                receiver.clear_cs_interrupt();
                match result {
                    Ok(l) => writeln!(tx, "received {} bytes", l).ok(),
                    Err(e) => writeln!(tx, "frame error: {}", e).ok(),
                };
            });
        });
    }

    #[task(binds = EXTI0, priority = 3, local = [button_pin, transmitter], shared = [uart_tx])]
    fn task_btn(mut ctx: task_btn::Context) {
        ctx.local.button_pin.clear_interrupt_pending_bit();

        ctx.shared.uart_tx.lock(|uart_tx| {
            writeln!(uart_tx, "Button is pressed!").ok();
        });
        let frame = [
            0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x90, 0x00,
            0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x11, 0x12, 0x13, 0x14, 0x15,
            0x16, 0x17, 0x18, 0x19, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x31,
            0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39,
        ];
        ctx.local.transmitter.transmit(frame.len(), |dst| {
            dst.copy_from_slice(&frame);
        });
    }
}

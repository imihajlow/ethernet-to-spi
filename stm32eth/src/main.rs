#![no_std]
#![no_main]

mod device;
mod receiver;
mod transmitter;
mod tx_frame_buf;

use panic_halt as _;

#[rtic::app(device = stm32f1xx_hal::pac, dispatchers = [ADC])]
mod app {
    use crate::device::ReceiverMutex;
    use cortex_m::{interrupt, interrupt::Mutex, singleton};
    use systick_monotonic::Systick;

    use smoltcp::{
        iface::{InterfaceBuilder, NeighborCache, SocketStorage},
        time::Instant,
        wire::{EthernetAddress, IpAddress, IpCidr},
    };
    use stm32f1xx_hal::{
        gpio::{Edge, ExtiPin, PA0},
        pac::USART3,
        prelude::*,
        serial::{Config, Serial, Tx},
    };

    use core::{cell::RefCell, fmt::Write};

    use crate::{device::SpiDevice, receiver::Receiver, transmitter::Transmitter};

    #[monotonic(binds = SysTick, default = true)]
    type MyMono = Systick<1000>; // 1000 Hz

    #[shared]
    struct Shared {
        uart_tx: Tx<USART3>,
    }

    #[local]
    struct Local {
        button_pin: PA0,
        transmitter: Option<Transmitter>,
        receiver_idle: ReceiverMutex,
        receiver_cs_down: ReceiverMutex,
    }

    #[init(local = [receiver: Mutex<RefCell<Option<Receiver>>> = Mutex::new(RefCell::new(None))])]
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

        let mono = Systick::new(cx.core.SYST, 40_000_000);

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
        interrupt::free(|cs| cx.local.receiver.borrow(cs).borrow_mut().replace(receiver));

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
            Shared { uart_tx: uart_tx },
            Local {
                button_pin: button_pin,
                transmitter: Some(transmitter),
                receiver_idle: cx.local.receiver,
                receiver_cs_down: cx.local.receiver,
            },
            init::Monotonics(mono),
        )
    }

    #[idle(shared = [uart_tx], local = [receiver_idle, transmitter])]
    fn idle(mut ctx: idle::Context) -> ! {
        let receiver = ctx.local.receiver_idle;
        interrupt::free(|cs| {
            receiver
                .borrow(cs)
                .borrow_mut()
                .as_mut()
                .unwrap()
                .start_listen()
        });

        let device = SpiDevice::new(receiver, ctx.local.transmitter.take().unwrap());
        let mut sockets = [SocketStorage::EMPTY; 2];
        let mut neighbor_cache_storage = [None; 8];
        let hwaddr = EthernetAddress([0x11, 0x22, 0x33, 0x44, 0x55, 0x66]);
        let neighbor_cache = NeighborCache::new(&mut neighbor_cache_storage[..]);
        let ip_addr = IpCidr::new(IpAddress::v4(10, 0, 0, 100), 24);
        let mut ip_addr_storage = [ip_addr; 1];

        let mut iface = InterfaceBuilder::new(device, &mut sockets[..])
            .hardware_addr(hwaddr.into())
            .neighbor_cache(neighbor_cache)
            .ip_addrs(&mut ip_addr_storage[..])
            .finalize();

        loop {
            let now = Instant::from_millis(monotonics::now().ticks() as i64);
            iface.poll(now).ok();
        }
    }

    #[task(binds = EXTI9_5, priority = 3, shared = [uart_tx], local = [receiver_cs_down])]
    fn task_cs_down(mut ctx: task_cs_down::Context) {
        let receiver = ctx.local.receiver_cs_down;

        ctx.shared.uart_tx.lock(|tx| {
            writeln!(tx, "cs down").ok();
            interrupt::free(|cs| {
                let result = receiver
                    .borrow(cs)
                    .borrow_mut()
                    .as_mut()
                    .map(|r| {
                        let result = r.on_frame_end();
                        r.clear_cs_interrupt();
                        result
                    })
                    .unwrap();
                match result {
                    Ok(l) => writeln!(tx, "received {} bytes", l).ok(),
                    Err(e) => writeln!(tx, "frame error: {}", e).ok(),
                };
            });
        });
    }

    #[task(binds = EXTI0, priority = 3, local = [button_pin], shared = [uart_tx])]
    fn task_btn(mut ctx: task_btn::Context) {
        ctx.local.button_pin.clear_interrupt_pending_bit();

        ctx.shared.uart_tx.lock(|uart_tx| {
            writeln!(uart_tx, "Button is pressed!").ok();
        });
    }
}

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

mod device;
mod receiver;
// mod server;
mod adapter;
mod bot_token;
mod event;
mod tg;
mod tg_bot;
mod tls_task;
mod transmitter;
mod tx_frame_buf;

use defmt_rtt as _;
use panic_probe as _;

#[rtic::app(device = stm32f4xx_hal::pac, dispatchers = [ADC])]
mod app {
    use crate::adapter::TcpSocketAdapter;
    use crate::device::{ReceiverMutex, SpiDevice};
    use crate::receiver::Receiver;
    use crate::tls_task;
    use crate::transmitter::Transmitter;
    use core::cell::RefCell;
    use core::sync::atomic::{AtomicUsize, Ordering};
    use core::task::{Context, Poll};
    use cortex_m::{interrupt, interrupt::Mutex, singleton};
    use futures::task::noop_waker_ref;
    use futures::Future;
    use rand::rngs::StdRng;
    use rand::RngCore;
    use rand::SeedableRng;
    use smoltcp::iface::Interface;
    use smoltcp::iface::InterfaceBuilder;
    use smoltcp::iface::NeighborCache;
    use smoltcp::iface::Routes;
    use smoltcp::iface::SocketStorage;
    use smoltcp::phy::Device;
    use smoltcp::socket::TcpSocket;
    use smoltcp::socket::TcpSocketBuffer;
    use smoltcp::socket::{Dhcpv4Event, Dhcpv4Socket};
    use smoltcp::time::Duration;
    use smoltcp::time::Instant;
    use smoltcp::wire::EthernetAddress;
    use smoltcp::wire::IpCidr;
    use smoltcp::wire::Ipv4Address;
    use smoltcp::wire::Ipv4Cidr;
    use stm32f4xx_hal::dma::StreamsTuple;
    use stm32f4xx_hal::gpio::{Edge, Output};
    use stm32f4xx_hal::{
        gpio::{PinState, PA5, PC13, PC4},
        prelude::*,
    };
    use systick_monotonic::Systick;

    #[monotonic(binds = SysTick, default = true)]
    type MyMono = Systick<1000>; // 1000 Hz

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        led_pin: PA5<Output>,
        button_pin: PC13,
        out_pin: PC4<Output>,
        transmitter: Option<Transmitter>,
        receiver_idle: ReceiverMutex,
        receiver_cs_up: ReceiverMutex,
        bt_press_producer: heapless::spsc::Producer<'static, (), 4>,
        bt_press_consumer: heapless::spsc::Consumer<'static, (), 4>,
    }

    static BUTTON_PRESS_COUNT: AtomicUsize = AtomicUsize::new(0);

    #[init(local = [
        receiver: Mutex<RefCell<Option<Receiver>>> = Mutex::new(RefCell::new(None)),
        bt_press_queue: heapless::spsc::Queue<(), 4> = heapless::spsc::Queue::new()])]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        let mut dp = cx.device;

        let rcc = dp.RCC.constrain();
        let gpioa = dp.GPIOA.split();
        let gpiob = dp.GPIOB.split();
        let gpioc = dp.GPIOC.split();

        let clocks = rcc.cfgr.sysclk(80.MHz()).pclk1(40.MHz()).freeze();
        let mut syscfg = dp.SYSCFG.constrain();

        let mono = Systick::new(cx.core.SYST, 40_000_000);

        let led_pin = gpioa.pa5.into_push_pull_output_in_state(PinState::High);
        let mut button_pin = gpioc.pc13.into_input();
        button_pin.make_interrupt_source(&mut syscfg);
        button_pin.enable_interrupt(&mut dp.EXTI);
        button_pin.trigger_on_edge(&mut dp.EXTI, Edge::RisingFalling);
        let out_pin = gpioc.pc4.into_push_pull_output();

        let pin_sck = gpiob.pb13;
        let pin_mosi = gpiob.pb15;
        let mut pin_cs = gpioc.pc6.into_pull_down_input();
        pin_cs.make_interrupt_source(&mut syscfg);
        pin_cs.enable_interrupt(&mut dp.EXTI);
        pin_cs.trigger_on_edge(&mut dp.EXTI, Edge::Rising);

        let streams_dma1 = StreamsTuple::new(dp.DMA1);
        let rx_stream = streams_dma1.3;

        let rx_bufs = [
            singleton!(: [u8; crate::receiver::BUFFER_LEN] = [0; crate::receiver::BUFFER_LEN])
                .unwrap(),
            singleton!(: [u8; crate::receiver::BUFFER_LEN] = [0; crate::receiver::BUFFER_LEN])
                .unwrap(),
            singleton!(: [u8; crate::receiver::BUFFER_LEN] = [0; crate::receiver::BUFFER_LEN])
                .unwrap(),
            singleton!(: [u8; crate::receiver::BUFFER_LEN] = [0; crate::receiver::BUFFER_LEN])
                .unwrap(),
        ];
        let receiver = Receiver::new(
            dp.SPI2, pin_sck, pin_mosi, pin_cs, rx_stream, rx_bufs, &clocks,
        );
        interrupt::free(|cs| cx.local.receiver.borrow(cs).borrow_mut().replace(receiver));

        let pin_tx_sck = gpioc.pc10.into_push_pull_output();
        let pin_tx_mosi = gpioc.pc12.into_push_pull_output();

        let tx_buf = singleton!(: [u8; crate::transmitter::BUFFER_LEN] = [0; crate::transmitter::BUFFER_LEN]).unwrap();
        let tx_stream = streams_dma1.7;

        let transmitter = Transmitter::new(
            false,
            false,
            false,
            dp.SPI3,
            pin_tx_sck,
            pin_tx_mosi,
            tx_stream,
            &clocks,
            tx_buf,
        );
        let (producer, consumer) = cx.local.bt_press_queue.split();
        (
            Shared {},
            Local {
                led_pin,
                button_pin,
                out_pin,
                receiver_idle: cx.local.receiver,
                receiver_cs_up: cx.local.receiver,
                transmitter: Some(transmitter),
                bt_press_producer: producer,
                bt_press_consumer: consumer,
            },
            init::Monotonics(mono),
        )
    }

    #[task(binds = EXTI15_10, priority = 5, local = [button_pin, out_pin, led_pin, bt_press_producer])]
    fn task_exti15_10(ctx: task_exti15_10::Context) {
        let button_pin = ctx.local.button_pin;
        let out_pin = ctx.local.out_pin;
        let led_pin = ctx.local.led_pin;
        let producer = ctx.local.bt_press_producer;
        if button_pin.check_interrupt() {
            if button_pin.is_high() {
                out_pin.set_high();
                led_pin.set_high();
            } else {
                BUTTON_PRESS_COUNT.fetch_add(1, Ordering::SeqCst);
                producer.enqueue(()).ok();
                out_pin.set_low();
                led_pin.set_low();
            }
            button_pin.clear_interrupt_pending_bit();
        }
    }

    #[idle(local = [receiver_idle, transmitter, bt_press_consumer])]
    fn idle(ctx: idle::Context) -> ! {
        defmt::trace!("idle task started");
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
        let mut sockets = [SocketStorage::EMPTY; 3];
        let mut neighbor_cache_storage = [None; 8];
        let hwaddr = EthernetAddress([0x06, 0x22, 0x33, 0x44, 0x55, 0x66]);
        let neighbor_cache = NeighborCache::new(&mut neighbor_cache_storage[..]);
        let ip_addr = IpCidr::new(Ipv4Address::UNSPECIFIED.into(), 0);
        let mut ip_addr_storage = [ip_addr; 1];
        let mut routes_storage = [None; 2];
        let routes = Routes::new(&mut routes_storage[..]);

        let mut iface = InterfaceBuilder::new(device, &mut sockets[..])
            .hardware_addr(hwaddr.into())
            .neighbor_cache(neighbor_cache)
            .ip_addrs(&mut ip_addr_storage[..])
            .routes(routes)
            .finalize();

        let mut dhcp_socket = Dhcpv4Socket::new();
        dhcp_socket.set_max_lease_duration(Some(Duration::from_secs(86400)));

        let dhcp_handle = iface.add_socket(dhcp_socket);

        let tcp_rx_buffer_storage_1 = singleton!(: [u8; 4096] = [0; 4096]).unwrap();
        let tcp_tx_buffer_storage_1 = singleton!(: [u8; 4096] = [0; 4096]).unwrap();
        let tcp_rx_buffer_1 = TcpSocketBuffer::new(&mut tcp_rx_buffer_storage_1[..]);
        let tcp_tx_buffer_1 = TcpSocketBuffer::new(&mut tcp_tx_buffer_storage_1[..]);
        let tcp_socket_1 = TcpSocket::new(tcp_rx_buffer_1, tcp_tx_buffer_1);

        let tcp_rx_buffer_storage_2 = singleton!(: [u8; 4096] = [0; 4096]).unwrap();
        let tcp_tx_buffer_storage_2 = singleton!(: [u8; 4096] = [0; 4096]).unwrap();
        let tcp_rx_buffer_2 = TcpSocketBuffer::new(&mut tcp_rx_buffer_storage_2[..]);
        let tcp_tx_buffer_2 = TcpSocketBuffer::new(&mut tcp_tx_buffer_storage_2[..]);
        let tcp_socket_2 = TcpSocket::new(tcp_rx_buffer_2, tcp_tx_buffer_2);

        let tcp_handle_1 = iface.add_socket(tcp_socket_1);
        let tcp_handle_2 = iface.add_socket(tcp_socket_2);

        defmt::trace!("fuck");
        let mut dhcp_ok = false;
        while !dhcp_ok {
            let now = Instant::from_millis(monotonics::now().ticks() as i64);
            match iface.poll(now) {
                Ok(_) => (),
                Err(smoltcp::Error::Unrecognized) => (),
                Err(e) => {
                    defmt::error!("poll error {}", e);
                }
            };

            let event = iface.get_socket::<Dhcpv4Socket>(dhcp_handle).poll();
            match event {
                None => {}
                Some(Dhcpv4Event::Configured(config)) => {
                    defmt::debug!("DHCP config acquired!");

                    defmt::debug!("IP address:      {}", config.address);
                    set_ipv4_addr(&mut iface, config.address);

                    if let Some(router) = config.router {
                        defmt::debug!("Default gateway: {}", router);
                        iface.routes_mut().add_default_ipv4_route(router).unwrap();
                    } else {
                        defmt::debug!("Default gateway: None");
                        iface.routes_mut().remove_default_ipv4_route();
                    }

                    for (i, s) in config.dns_servers.iter().enumerate() {
                        if let Some(s) = s {
                            defmt::debug!("DNS server {}:    {}", i, s);
                        }
                    }
                    dhcp_ok = true;
                }
                Some(Dhcpv4Event::Deconfigured) => {
                    defmt::debug!("DHCP lost config!");
                    set_ipv4_addr(&mut iface, Ipv4Cidr::new(Ipv4Address::UNSPECIFIED, 0));
                    iface.routes_mut().remove_default_ipv4_route();
                }
            }
        }

        defmt::info!("now = {}", monotonics::now().ticks());
        let mut rng1 = StdRng::seed_from_u64(monotonics::now().ticks() as u64);
        let mut rng2 = StdRng::seed_from_u64(rng1.next_u64());

        let iface_cell = RefCell::new(iface);
        let adapter1 = TcpSocketAdapter::new(&iface_cell, tcp_handle_1, || {
            monotonics::now().ticks() as i64
        });
        let adapter2 = TcpSocketAdapter::new(&iface_cell, tcp_handle_2, || {
            monotonics::now().ticks() as i64
        });
        {
            let mut task = tls_task::bot_task(
                adapter1,
                adapter2,
                &mut rng1,
                &mut rng2,
                ctx.local.bt_press_consumer,
            );
            let mut task_pin = unsafe { core::pin::Pin::new_unchecked(&mut task) };
            let mut ctx = Context::from_waker(noop_waker_ref());
            let mut result = Poll::Pending;
            while result.is_pending() {
                result = task_pin.as_mut().poll(&mut ctx);
            }
        };
        unreachable!();
    }

    #[task(binds = EXTI9_5, priority = 3, local = [receiver_cs_up])]
    fn task_cs_up(ctx: task_cs_up::Context) {
        let receiver = ctx.local.receiver_cs_up;

        interrupt::free(|cs| {
            let result = receiver
                .borrow(cs)
                .borrow_mut()
                .as_mut()
                .map(|r| {
                    let result = r.on_edge_maybe();
                    result
                })
                .unwrap();
            if let Some(result) = result {
                match result {
                    Ok(l) => defmt::info!("received {} bytes", l),
                    Err(e) => defmt::error!("frame error: {}", e),
                }
            }
        });
    }

    fn set_ipv4_addr<DeviceT>(iface: &mut Interface<'_, DeviceT>, cidr: Ipv4Cidr)
    where
        DeviceT: for<'d> Device<'d>,
    {
        iface.update_ip_addrs(|addrs| {
            let dest = addrs.iter_mut().next().unwrap();
            *dest = IpCidr::Ipv4(cidr);
        });
    }
}

#![no_std]
#![no_main]

mod device;
mod receiver;
// mod server;
mod transmitter;
mod tx_frame_buf;

use panic_halt as _;

#[rtic::app(device = stm32f1xx_hal::pac, dispatchers = [ADC])]
mod app {
    use crate::device::ReceiverMutex;
    use core::fmt::Write;
    use core::sync::atomic::{AtomicUsize, Ordering};
    // use crate::server::Server;
    use cortex_m::{interrupt, interrupt::Mutex, singleton};
    use httparse::Status;
    use smoltcp::iface::Interface;
    use smoltcp::iface::Routes;
    use smoltcp::phy::Device;
    use smoltcp::socket::TcpSocket;
    use smoltcp::socket::TcpSocketBuffer;
    use smoltcp::socket::{Dhcpv4Event, Dhcpv4Socket};
    use smoltcp::time::Duration;
    use smoltcp::wire::Ipv4Address;
    use smoltcp::wire::Ipv4Cidr;
    use systick_monotonic::Systick;

    use smoltcp::{
        iface::{InterfaceBuilder, NeighborCache, SocketStorage},
        time::Instant,
        wire::{EthernetAddress, IpCidr},
    };
    use stm32f1xx_hal::{
        gpio::{Edge, ExtiPin, PA0},
        pac::USART3,
        prelude::*,
        serial::{Config, Serial, Tx},
    };

    use core::cell::RefCell;

    use crate::{device::SpiDevice, receiver::Receiver, transmitter::Transmitter};

    #[defmt::global_logger]
    struct Logger;

    static mut UART_TX: Option<Tx<USART3>> = None;

    unsafe impl defmt::Logger for Logger {
        fn acquire() {
            cortex_m::interrupt::disable();
        }

        unsafe fn flush() {
            if let Some(tx) = &mut UART_TX {
                tx.flush().ok();
            }
        }

        unsafe fn release() {
            cortex_m::interrupt::enable();
        }

        unsafe fn write(bytes: &[u8]) {
            if let Some(tx) = &mut UART_TX {
                tx.bwrite_all(bytes).ok();
            }
        }
    }

    #[monotonic(binds = SysTick, default = true)]
    type MyMono = Systick<1000>; // 1000 Hz

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        button_pin: PA0,
        transmitter: Option<Transmitter>,
        receiver_idle: ReceiverMutex,
        receiver_cs_down: ReceiverMutex,
    }

    static BUTTON_PRESS_COUNT: AtomicUsize = AtomicUsize::new(0);

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
        unsafe {
            UART_TX.replace(uart_tx);
        }

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
            true,
            true,
            true,
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
            Shared {},
            Local {
                button_pin: button_pin,
                transmitter: Some(transmitter),
                receiver_idle: cx.local.receiver,
                receiver_cs_down: cx.local.receiver,
            },
            init::Monotonics(mono),
        )
    }

    #[idle(local = [receiver_idle, transmitter, ])]
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
        let mut sockets = [SocketStorage::EMPTY; 2];
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

        // let mut tcp_rx_buffer_storage = [0 as u8; 512];
        // let mut tcp_tx_buffer_storage = [0 as u8; 512];
        // let tcp_rx_buffer = TcpSocketBuffer::new(&mut tcp_rx_buffer_storage[..]);
        // let tcp_tx_buffer = TcpSocketBuffer::new(&mut tcp_tx_buffer_storage[..]);
        // let tcp_socket = TcpSocket::new(tcp_rx_buffer, tcp_tx_buffer);

        // let tcp_handle = iface.add_socket(tcp_socket);
        loop {
            defmt::info!("iteration");
            let mut request_done = false;
            // let mut buf = [0 as u8; 384];
            // let mut buf_slice = Some(&mut buf[..]);
            // let mut headers = [httparse::EMPTY_HEADER; 16];
            // let mut req = httparse::Request::new(&mut headers);

            while !request_done {
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
                    }
                    Some(Dhcpv4Event::Deconfigured) => {
                        defmt::debug!("DHCP lost config!");
                        set_ipv4_addr(&mut iface, Ipv4Cidr::new(Ipv4Address::UNSPECIFIED, 0));
                        iface.routes_mut().remove_default_ipv4_route();
                    }
                }

                // let socket = iface.get_socket::<TcpSocket>(tcp_handle);
                // if !socket.is_open() {
                //     socket.listen(80).ok();
                // }
                // if socket.may_recv() {
                //     let r = socket.recv(|buffer| {
                //         let recvd_len = buffer.len();
                //         let slice = buf_slice.take().unwrap();
                //         let r = if recvd_len > slice.len() {
                //             None
                //         } else {
                //             let (dst, rest) = slice.split_at_mut(recvd_len);
                //             dst.copy_from_slice(buffer);
                //             Some((dst, rest))
                //         };
                //         (recvd_len, r)
                //     });
                //     request_done = match r {
                //         Err(e) => {
                //             // 500
                //             defmt::error!("error in socket {}", e);
                //             write!(socket, "HTTP/1.1 500 Internal Server Error\r\nContent-Type: text/plain\r\nConnection: close\r\n\r\nError in socket").ok();
                //             true
                //         }
                //         Ok(None) => {
                //             // 500
                //             defmt::error!("buffer overrun");
                //             write!(socket, "HTTP/1.1 500 Internal Server Error\r\nContent-Type: text/plain\r\nConnection: close\r\n\r\nBuffer overrun").ok();
                //             true
                //         }
                //         Ok(Some((dst, rest))) => {
                //             buf_slice.replace(rest);
                //             let r = req.parse(dst);
                //             match r {
                //                 Err(_) => {
                //                     defmt::error!("error in request");
                //                     write!(socket, "HTTP/1.1 400 Bad Request\r\nContent-Type: text/plain\r\nConnection: close\r\n\r\nBad Request").ok();
                //                     true
                //                 }
                //                 Ok(Status::Partial) => false,
                //                 Ok(Status::Complete(_)) => {
                //                     // ok
                //                     defmt::info!("requested method: {}", req.method);
                //                     defmt::info!("requested path: {}", req.path);
                //                     if req.method == Some("GET") && req.path.is_some() {
                //                         write_response(socket, req.path.unwrap()).ok();
                //                         true
                //                     } else {
                //                         // 405
                //                         write!(socket, "HTTP/1.1 405 Method Not Allowed\r\nContent-Type: text/plain\r\nConnection: close\r\n\r\nMethod Not Allowed").ok();
                //                         true
                //                     }
                //                 }
                //             }
                //         }
                //     };
                //     if request_done {
                //         socket.close();
                //     }
                // }
            }
        }
    }

    #[task(binds = EXTI9_5, priority = 3, local = [receiver_cs_down])]
    fn task_cs_down(ctx: task_cs_down::Context) {
        let receiver = ctx.local.receiver_cs_down;

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
                Ok(l) => defmt::info!("received {} bytes", l),
                Err(e) => defmt::error!("frame error: {}", e),
            };
        });
    }

    #[task(binds = EXTI0, priority = 3, local = [button_pin])]
    fn task_btn(ctx: task_btn::Context) {
        ctx.local.button_pin.clear_interrupt_pending_bit();
        let prev = BUTTON_PRESS_COUNT.fetch_add(1, Ordering::SeqCst);

        defmt::debug!("Button is pressed! {}", prev);
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

    fn write_response<W: core::fmt::Write>(socket: &mut W, path: &str) -> core::fmt::Result {
        let r = match path {
            "/" =>
                socket.write_str(concat!("HTTP/1.1 200 Ok\r\nContent-Type: text/html\r\nConnection: close\r\n\r\n",
                    include_str!("index.html"))),
            "/button.txt" => {
                let press_count = BUTTON_PRESS_COUNT.load(Ordering::SeqCst);
                defmt::info!("get button.txt {}", press_count);
                socket.write_str("HTTP/1.1 200 Ok\r\nContent-Type: text/plain\r\nCache-Control: no-store\r\nConnection: close\r\n\r\n")?;
                write!(socket, "{}", press_count)
            }
            _ =>
                socket.write_str("HTTP/1.1 404 Not Found\r\nContent-Type: text/plain\r\nConnection: close\r\n\r\nNot found")
        };
        if r.is_err() {
            defmt::error!("write error!");
        }
        r
    }
}

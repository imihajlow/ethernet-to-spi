#![no_std]
#![no_main]

// mod device;
mod receiver;
// mod server;
mod transmitter;
mod tx_frame_buf;

use defmt_rtt as _;
use panic_halt as _;

#[rtic::app(device = stm32f4xx_hal::pac, dispatchers = [ADC])]
mod app {
    // use crate::device::ReceiverMutex;
    use core::fmt::Write;
    use core::sync::atomic::{AtomicUsize, Ordering};
    // use crate::server::Server;
    use cortex_m::{interrupt, interrupt::Mutex, singleton};
    use stm32f4xx_hal::dma::config::DmaConfig;
    use stm32f4xx_hal::dma::traits::Stream;
    use stm32f4xx_hal::dma::{Stream3, StreamsTuple, Transfer};
    use stm32f4xx_hal::gpio::{Edge, Input, NoPin, Output, PushPull};
    use stm32f4xx_hal::pac::{DMA1, SPI2, TIM2};
    use stm32f4xx_hal::spi::{Mode, NoMiso, Phase, Polarity, Slave, Spi, BitFormat};
    use stm32f4xx_hal::timer::{CounterMs, Event};
    // use httparse::Status;
    // use smoltcp::iface::Interface;
    // use smoltcp::iface::Routes;
    // use smoltcp::phy::Device;
    // use smoltcp::socket::TcpSocket;
    // use smoltcp::socket::TcpSocketBuffer;
    // use smoltcp::socket::{Dhcpv4Event, Dhcpv4Socket};
    // use smoltcp::time::Duration;
    // use smoltcp::wire::Ipv4Address;
    // use smoltcp::wire::Ipv4Cidr;
    use nb::block;
    use systick_monotonic::Systick;

    // use smoltcp::{
    //     iface::{InterfaceBuilder, NeighborCache, SocketStorage},
    //     time::Instant,
    //     wire::{EthernetAddress, IpCidr},
    // };
    // use stm32f1xx_hal::{
    //     gpio::{Edge, ExtiPin, PA0},
    //     pac::USART3,
    //     prelude::*,
    //     serial::{Config, Serial, Tx},
    // };

    use stm32f4xx_hal::{
        gpio::{PinState, PA5, PB13, PB15, PB9, PC13, PC4, PC6},
        prelude::*,
    };

    use core::cell::RefCell;

    // use crate::{device::SpiDevice, receiver::Receiver, transmitter::Transmitter};

    #[monotonic(binds = SysTick, default = true)]
    type MyMono = Systick<1000>; // 1000 Hz

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        led_pin: PA5<Output>,
        spi: Option<Spi<SPI2, (PB13, NoPin, PB15), false, u8, Slave>>,
        rx_stream: Option<Stream3<DMA1>>,
        cs_pin: PB9<Input>,
        button_pin: PC13,
        out_pin: PC4<Output>,
        // transmitter: Option<Transmitter>,
        // receiver_idle: ReceiverMutex,
        // receiver_cs_down: ReceiverMutex,
    }

    static BUTTON_PRESS_COUNT: AtomicUsize = AtomicUsize::new(0);

    // #[init(local = [receiver: Mutex<RefCell<Option<Receiver>>> = Mutex::new(RefCell::new(None))])]
    #[init(local = [])]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        let mut dp = cx.device;

        let rcc = dp.RCC.constrain();
        let mut gpioc = dp.GPIOC.split();

        let mut gpiob = dp.GPIOB.split();
        let mut gpioa = dp.GPIOA.split();
        let spi = dp.SPI2;

        let clocks = rcc.cfgr.sysclk(40.MHz()).pclk1(20.MHz()).freeze();
        let mut syscfg = dp.SYSCFG.constrain();

        let mono = Systick::new(cx.core.SYST, 40_000_000);

        let led_pin = gpioa.pa5.into_push_pull_output_in_state(PinState::High);
        let cs_pin = gpiob.pb9.into_input();
        let mut button_pin = gpioc.pc13.into_input();
        button_pin.make_interrupt_source(&mut syscfg);
        button_pin.enable_interrupt(&mut dp.EXTI);
        button_pin.trigger_on_edge(&mut dp.EXTI, Edge::RisingFalling);
        let out_pin = gpioc.pc4.into_push_pull_output();

        let sck = gpiob.pb13;
        let mosi = gpiob.pb15;
        let mut spi = Spi::new_slave(
            spi,
            (sck, NoMiso {}, mosi),
            Mode {
                polarity: Polarity::IdleLow,
                phase: Phase::CaptureOnFirstTransition,
            },
            10_000_000.Hz(),
            &clocks,
        );
        spi.bit_format(BitFormat::LsbFirst);
        let streams = StreamsTuple::new(dp.DMA1);
        let rx_stream = streams.3;
        (
            Shared {},
            Local {
                led_pin,
                spi: Some(spi),
                rx_stream: Some(rx_stream),
                cs_pin,
                button_pin,
                out_pin,
            },
            init::Monotonics(mono),
        )
    }

    #[task(binds = EXTI15_10, priority = 5, local = [button_pin, out_pin, led_pin])]
    fn task_exti15_10(ctx: task_exti15_10::Context) {
        let button_pin = ctx.local.button_pin;
        let out_pin = ctx.local.out_pin;
        let led_pin = ctx.local.led_pin;
        if button_pin.check_interrupt() {
            if button_pin.is_high() {
                out_pin.set_high();
                led_pin.set_high();
            } else {
                out_pin.set_low();
                led_pin.set_low();
            }
            button_pin.clear_interrupt_pending_bit();
        }
    }

    #[idle(local = [spi, rx_stream, cs_pin])]
    fn idle(ctx: idle::Context) -> ! {
        defmt::trace!("idle task started");
        let mut spi = ctx.local.spi.take().unwrap();
        let mut rx_buffer = cortex_m::singleton!(: [u8; 16] = [0; 16]).unwrap();
        let mut rx_stream = ctx.local.rx_stream.take().unwrap();
        let cs_pin = ctx.local.cs_pin;
        loop {
            (spi, rx_stream, rx_buffer) = {
                spi.set_internal_nss(false);
                let rx = spi.use_dma().rx();
                let mut rx_transfer = Transfer::init_peripheral_to_memory(
                    rx_stream,
                    rx,
                    rx_buffer,
                    None,
                    DmaConfig::default()
                        .memory_increment(true),
                );
                rx_transfer.start(|_rx| {});
                // defmt::info!("rxne = {}", spi.is_rx_not_empty());
                while cs_pin.is_high() {}
                while cs_pin.is_low() {}
                // defmt::info!("rxne = {}", spi.is_rx_not_empty());
                // match spi.check_read() {
                //     Ok(b) => defmt::info!("read {}", b),
                //     Err(e) => defmt::info!("error")
                // }
                let (stream, spi, buf, _) = rx_transfer.release();
                let stream: Stream3<DMA1> = stream;
                let ndtr = Stream3::<DMA1>::get_number_of_transfers() as usize;
                defmt::info!("ndtr = {}", ndtr);
                defmt::info!("data: {}", buf);
                let mut spi = spi.release();
                spi.set_internal_nss(true);
                defmt::info!("rxne = {}", spi.is_rx_not_empty());
                (spi, stream, buf)
            }

            // defmt::trace!("Got data: {:02X}", r)
        }
        // let receiver = ctx.local.receiver_idle;
        // interrupt::free(|cs| {
        //     receiver
        //         .borrow(cs)
        //         .borrow_mut()
        //         .as_mut()
        //         .unwrap()
        //         .start_listen()
        // });

        // let device = SpiDevice::new(receiver, ctx.local.transmitter.take().unwrap());
        // let mut sockets = [SocketStorage::EMPTY; 2];
        // let mut neighbor_cache_storage = [None; 8];
        // let hwaddr = EthernetAddress([0x06, 0x22, 0x33, 0x44, 0x55, 0x66]);
        // let neighbor_cache = NeighborCache::new(&mut neighbor_cache_storage[..]);
        // let ip_addr = IpCidr::new(Ipv4Address::UNSPECIFIED.into(), 0);
        // let mut ip_addr_storage = [ip_addr; 1];
        // let mut routes_storage = [None; 2];
        // let routes = Routes::new(&mut routes_storage[..]);

        // let mut iface = InterfaceBuilder::new(device, &mut sockets[..])
        //     .hardware_addr(hwaddr.into())
        //     .neighbor_cache(neighbor_cache)
        //     .ip_addrs(&mut ip_addr_storage[..])
        //     .routes(routes)
        //     .finalize();

        // let mut dhcp_socket = Dhcpv4Socket::new();
        // dhcp_socket.set_max_lease_duration(Some(Duration::from_secs(86400)));

        // let dhcp_handle = iface.add_socket(dhcp_socket);

        // let mut tcp_rx_buffer_storage = [0 as u8; 512];
        // let mut tcp_tx_buffer_storage = [0 as u8; 512];
        // let tcp_rx_buffer = TcpSocketBuffer::new(&mut tcp_rx_buffer_storage[..]);
        // let tcp_tx_buffer = TcpSocketBuffer::new(&mut tcp_tx_buffer_storage[..]);
        // let tcp_socket = TcpSocket::new(tcp_rx_buffer, tcp_tx_buffer);

        // let tcp_handle = iface.add_socket(tcp_socket);
        // loop {
        //     defmt::info!("iteration");
        //     let mut request_done = false;
        //     // let mut buf = [0 as u8; 384];
        //     // let mut buf_slice = Some(&mut buf[..]);
        //     // let mut headers = [httparse::EMPTY_HEADER; 16];
        //     // let mut req = httparse::Request::new(&mut headers);

        //     while !request_done {
        //         let now = Instant::from_millis(monotonics::now().ticks() as i64);
        //         match iface.poll(now) {
        //             Ok(_) => (),
        //             Err(smoltcp::Error::Unrecognized) => (),
        //             Err(e) => {
        //                 defmt::error!("poll error {}", e);
        //             }
        //         };

        //         let event = iface.get_socket::<Dhcpv4Socket>(dhcp_handle).poll();
        //         match event {
        //             None => {}
        //             Some(Dhcpv4Event::Configured(config)) => {
        //                 defmt::debug!("DHCP config acquired!");

        //                 defmt::debug!("IP address:      {}", config.address);
        //                 set_ipv4_addr(&mut iface, config.address);

        //                 if let Some(router) = config.router {
        //                     defmt::debug!("Default gateway: {}", router);
        //                     iface.routes_mut().add_default_ipv4_route(router).unwrap();
        //                 } else {
        //                     defmt::debug!("Default gateway: None");
        //                     iface.routes_mut().remove_default_ipv4_route();
        //                 }

        //                 for (i, s) in config.dns_servers.iter().enumerate() {
        //                     if let Some(s) = s {
        //                         defmt::debug!("DNS server {}:    {}", i, s);
        //                     }
        //                 }
        //             }
        //             Some(Dhcpv4Event::Deconfigured) => {
        //                 defmt::debug!("DHCP lost config!");
        //                 set_ipv4_addr(&mut iface, Ipv4Cidr::new(Ipv4Address::UNSPECIFIED, 0));
        //                 iface.routes_mut().remove_default_ipv4_route();
        //             }
        //         }

        //         // let socket = iface.get_socket::<TcpSocket>(tcp_handle);
        //         // if !socket.is_open() {
        //         //     socket.listen(80).ok();
        //         // }
        //         // if socket.may_recv() {
        //         //     let r = socket.recv(|buffer| {
        //         //         let recvd_len = buffer.len();
        //         //         let slice = buf_slice.take().unwrap();
        //         //         let r = if recvd_len > slice.len() {
        //         //             None
        //         //         } else {
        //         //             let (dst, rest) = slice.split_at_mut(recvd_len);
        //         //             dst.copy_from_slice(buffer);
        //         //             Some((dst, rest))
        //         //         };
        //         //         (recvd_len, r)
        //         //     });
        //         //     request_done = match r {
        //         //         Err(e) => {
        //         //             // 500
        //         //             defmt::error!("error in socket {}", e);
        //         //             write!(socket, "HTTP/1.1 500 Internal Server Error\r\nContent-Type: text/plain\r\nConnection: close\r\n\r\nError in socket").ok();
        //         //             true
        //         //         }
        //         //         Ok(None) => {
        //         //             // 500
        //         //             defmt::error!("buffer overrun");
        //         //             write!(socket, "HTTP/1.1 500 Internal Server Error\r\nContent-Type: text/plain\r\nConnection: close\r\n\r\nBuffer overrun").ok();
        //         //             true
        //         //         }
        //         //         Ok(Some((dst, rest))) => {
        //         //             buf_slice.replace(rest);
        //         //             let r = req.parse(dst);
        //         //             match r {
        //         //                 Err(_) => {
        //         //                     defmt::error!("error in request");
        //         //                     write!(socket, "HTTP/1.1 400 Bad Request\r\nContent-Type: text/plain\r\nConnection: close\r\n\r\nBad Request").ok();
        //         //                     true
        //         //                 }
        //         //                 Ok(Status::Partial) => false,
        //         //                 Ok(Status::Complete(_)) => {
        //         //                     // ok
        //         //                     defmt::info!("requested method: {}", req.method);
        //         //                     defmt::info!("requested path: {}", req.path);
        //         //                     if req.method == Some("GET") && req.path.is_some() {
        //         //                         write_response(socket, req.path.unwrap()).ok();
        //         //                         true
        //         //                     } else {
        //         //                         // 405
        //         //                         write!(socket, "HTTP/1.1 405 Method Not Allowed\r\nContent-Type: text/plain\r\nConnection: close\r\n\r\nMethod Not Allowed").ok();
        //         //                         true
        //         //                     }
        //         //                 }
        //         //             }
        //         //         }
        //         //     };
        //         //     if request_done {
        //         //         socket.close();
        //         //     }
        //         // }
        //     }
        // }
    }

    // #[task(binds = EXTI9_5, priority = 3, local = [receiver_cs_down])]
    // fn task_cs_down(ctx: task_cs_down::Context) {
    //     let receiver = ctx.local.receiver_cs_down;

    //     interrupt::free(|cs| {
    //         let result = receiver
    //             .borrow(cs)
    //             .borrow_mut()
    //             .as_mut()
    //             .map(|r| {
    //                 let result = r.on_frame_end();
    //                 r.clear_cs_interrupt();
    //                 result
    //             })
    //             .unwrap();
    //         match result {
    //             Ok(l) => defmt::info!("received {} bytes", l),
    //             Err(e) => defmt::error!("frame error: {}", e),
    //         };
    //     });
    // }

    // fn set_ipv4_addr<DeviceT>(iface: &mut Interface<'_, DeviceT>, cidr: Ipv4Cidr)
    // where
    //     DeviceT: for<'d> Device<'d>,
    // {
    //     iface.update_ip_addrs(|addrs| {
    //         let dest = addrs.iter_mut().next().unwrap();
    //         *dest = IpCidr::Ipv4(cidr);
    //     });
    // }

    // fn write_response<W: core::fmt::Write>(socket: &mut W, path: &str) -> core::fmt::Result {
    //     let r = match path {
    //         "/" =>
    //             socket.write_str(concat!("HTTP/1.1 200 Ok\r\nContent-Type: text/html\r\nConnection: close\r\n\r\n",
    //                 include_str!("index.html"))),
    //         "/button.txt" => {
    //             let press_count = BUTTON_PRESS_COUNT.load(Ordering::SeqCst);
    //             defmt::info!("get button.txt {}", press_count);
    //             socket.write_str("HTTP/1.1 200 Ok\r\nContent-Type: text/plain\r\nCache-Control: no-store\r\nConnection: close\r\n\r\n")?;
    //             write!(socket, "{}", press_count)
    //         }
    //         _ =>
    //             socket.write_str("HTTP/1.1 404 Not Found\r\nContent-Type: text/plain\r\nConnection: close\r\n\r\nNot found")
    //     };
    //     if r.is_err() {
    //         defmt::error!("write error!");
    //     }
    //     r
    // }
}

#![no_std]
#![no_main]

use cortex_m::singleton;
use cortex_m_rt::entry;
use embedded_hal::spi::{Mode, Phase, Polarity}; // The runtime
                                                // the `set_high/low`function
#[allow(unused_imports)]
use panic_halt;
use stm32f1xx_hal::{
    pac::{self},
    prelude::*,
    serial::{Config, Serial},
    spi::{NoMiso, Spi},
}; // STM32F1 specific functions // When a panic occurs, stop the microcontroller

use core::fmt::Write;

pub const MODE: Mode = Mode {
    phase: Phase::CaptureOnFirstTransition,
    polarity: Polarity::IdleLow,
};

#[entry]
fn main() -> ! {
    // Get handles to the hardware objects. These functions can only be called
    // once, so that the borrowchecker can ensure you don't reconfigure
    // something by accident.
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    let mut rcc = dp.RCC.constrain();
    let mut gpioc = dp.GPIOC.split();

    let mut afio = dp.AFIO.constrain();
    let mut gpiob = dp.GPIOB.split();

    let mut flash = dp.FLASH.constrain();
    let clocks = rcc.cfgr.sysclk(8.MHz()).freeze(&mut flash.acr);

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

    let (mut uart_tx, _rx) = serial.split();

    // SPI2
    let pin_sck = gpiob.pb13;
    let pin_mosi = gpiob.pb15;

    let dma = dp.DMA1.split();

    let mut buf = Some(singleton!(: [u8; 2048] = [0; 2048]).unwrap());
    let mut n = 0;
    let mut periph = Some((dp.SPI2, pin_sck, pin_mosi, dma.4));
    loop {
        // init SPI and start transfer regardless of CS
        let (spi, sck, mosi, dma) = periph.take().unwrap();
        let spi2 = Spi::spi2_slave(spi, (sck, NoMiso, mosi), MODE);
        let spi_dma_rx = spi2.with_rx_dma(dma);

        let transfer = {
            let buf = buf.take().unwrap();
            spi_dma_rx.read(buf)
        };
        writeln!(uart_tx, "transfer start {}", n);
        n += 1;
        // wait for CS to go low
        while !pin_cs.is_low() {}
        // wait for CS to go high
        while !pin_cs.is_high() {}

        // finish transfer
        {
            let (ret_buf, ret_rx) = transfer.stop();
            let ndtr = ret_rx.channel.get_ndtr() as usize;
            writeln!(uart_tx, "transfer end, ndtr = {}", ndtr);
            for i in 0..ret_buf.len() - ndtr {
                write!(uart_tx, "{:02X} ", ret_buf[i]);
            }
            uart_tx.write_char('\n');
            buf.replace(ret_buf);

            // release SPI to reset it
            let (spi, dma) = ret_rx.release();
            let (spi, (sck, _, mosi)) = spi.release();
            periph.replace((spi, sck, mosi, dma));
        }
    }
}


use replace_with::replace_with_and_return;
use stm32f4xx_hal::{
    dma::{config, Stream7, Transfer},
    gpio::{Output, PC10, PC12},
    pac::{DMA1, SPI3},
    prelude::*,
    rcc::Clocks,
    spi::{BitFormat, Mode, NoMiso, Phase, Polarity, Spi},
};

use crate::tx_frame_buf::TxFrameBuf;

pub const BUFFER_LEN: usize = 1600;
pub const MTU: usize = BUFFER_LEN - 8 - 4;

pub type TxBuf = &'static mut [u8; BUFFER_LEN];

type PinSck = PC10<Output>;
type PinMosi = PC12<Output>;
type DmaStream = Stream7<DMA1>;

pub struct Transmitter {
    spi: Spi<SPI3, (PinSck, NoMiso, PinMosi), false>,
    dma_stream: DmaStream,
    buf: TxBuf,
    invert_idle: bool,
    invert_data: bool,
    invert_sck_pol: bool,
}

impl Transmitter {
    pub fn new(
        invert_idle: bool,
        invert_data: bool,
        invert_sck_pol: bool,
        spi: SPI3,
        mut sck: PinSck,
        mut mosi: PinMosi,
        dma_stream: DmaStream,
        clocks: &Clocks,
        buf: TxBuf,
    ) -> Self {
        if invert_idle {
            sck.set_high();
        } else {
            sck.set_low();
        }
        mosi.set_low();
        let mode = Mode {
            polarity: Polarity::IdleLow,
            phase: Phase::CaptureOnSecondTransition,
        };
        let mut spi = Spi::new(spi, (sck, NoMiso {}, mosi), mode, 10.MHz(), clocks);
        spi.bit_format(BitFormat::LsbFirst);
        Self {
            spi,
            dma_stream,
            buf,
            invert_idle,
            invert_data,
            invert_sck_pol,
        }
    }

    pub fn transmit<R, F>(&mut self, len: usize, f: F) -> Option<R>
    where
        F: FnOnce(&mut [u8]) -> R,
    {
        replace_with_and_return(
            self,
            || panic!(),
            |s| {
                let Self {
                    spi,
                    dma_stream,
                    buf,
                    invert_idle,
                    invert_data,
                    invert_sck_pol,
                } = s;
                let (frame_buf, result) = TxFrameBuf::new_with_fn(buf, len, f, invert_data);

                let dma_tx = spi.use_dma().tx();
                let mut transfer = Transfer::init_memory_to_peripheral(
                    dma_stream,
                    dma_tx,
                    frame_buf,
                    None,
                    config::DmaConfig::default().memory_increment(true),
                );

                transfer.start(|_tx| {});

                transfer.wait();

                let (dma_stream, tx, buf, _) = transfer.release();
                let spi = tx.release();

                while spi.is_busy() {}

                (
                    Some(result),
                    Self {
                        spi,
                        dma_stream,
                        buf: buf.release(),
                        invert_idle,
                        invert_data,
                        invert_sck_pol,
                    },
                )
            },
        )
    }
}

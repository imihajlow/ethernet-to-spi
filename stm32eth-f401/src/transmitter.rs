use cortex_m::asm;
use replace_with::replace_with_and_return;
use stm32f4xx_hal::{
    gpio::{Output, PC10, PC12, PB1},
    pac::{SPI3, DMA1},
    prelude::*,
    rcc::Clocks,
    spi::{BitFormat, Mode, NoMiso, Phase, Polarity, Spi},
    dma::{Transfer, config, Stream7},
};

use crate::tx_frame_buf::TxFrameBuf;

pub const BUFFER_LEN: usize = 1600;
pub const MTU: usize = BUFFER_LEN - 8 - 4;

pub type TxBuf = &'static mut [u8; BUFFER_LEN];

type PinSck = PC10<Output>;
type PinMosi = PC12<Output>;
type PinNlpDisa = PB1<Output>;
type DmaStream = Stream7<DMA1>;

pub struct SpiTxPeriph {
    spi: SPI3,
    sck: PinSck,
    mosi: PinMosi,
    nlp_disa: PinNlpDisa,
    dma_stream: DmaStream,
    clocks: Clocks,
}

pub enum Transmitter {
    Idle {
        periph: SpiTxPeriph,
        buf: TxBuf,
        invert_idle: bool,
        invert_data: bool,
        invert_sck_pol: bool,
    },
    Invalid,
}

impl Transmitter {
    pub fn new(
        invert_idle: bool,
        invert_data: bool,
        invert_sck_pol: bool,
        spi: SPI3,
        mut sck: PinSck,
        mut mosi: PinMosi,
        mut nlp_disa: PinNlpDisa,
        dma_stream: DmaStream,
        clocks: Clocks,
        buf: TxBuf,
    ) -> Self {
        nlp_disa.set_low();

        if invert_idle {
            sck.set_low();
        } else {
            sck.set_high();
        }
        mosi.set_low();
        Self::Idle {
            periph: SpiTxPeriph {
                spi,
                sck,
                mosi,
                nlp_disa,
                dma_stream,
                clocks,
            },
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
            || Self::Invalid,
            |s| {
                if let Self::Idle {
                    mut periph,
                    buf,
                    invert_idle,
                    invert_data,
                    invert_sck_pol,
                } = s
                {
                    periph.nlp_disa.set_high();

                    let (frame_buf, result) = TxFrameBuf::new_with_fn(buf, len, f, invert_data);

                    let mode = Mode {
                        polarity: if invert_sck_pol {
                            Polarity::IdleHigh
                        } else {
                            Polarity::IdleLow
                        },
                        phase: Phase::CaptureOnFirstTransition,
                        // phase: Phase::CaptureOnSecondTransition,
                    };
                    let mut spi1 = Spi::new(
                        periph.spi,
                        (periph.sck, NoMiso {}, periph.mosi),
                        mode,
                        10.MHz(),
                        &periph.clocks,
                    );
                    spi1.bit_format(BitFormat::LsbFirst);
                    let dma_tx = spi1.use_dma().tx();
                    let mut transfer = Transfer::init_memory_to_peripheral(
                        periph.dma_stream,
                        dma_tx,
                        frame_buf,
                        None,
                        config::DmaConfig::default().memory_increment(true),
                    );

                    transfer.start(|_tx| {});

                    transfer.wait();

                    let (stream, tx, buf, _) = transfer.release();
                    let spi = tx.release();

                    periph.dma_stream = stream;

                    while spi.is_busy() {}

                    asm::delay(400);

                    let (spi, (sck, _, mosi)) = spi.release();
                    periph.spi = spi;
                    periph.sck = sck;
                    periph.mosi = mosi;
                    if invert_idle {
                        periph.sck.set_low();
                    } else {
                        periph.sck.set_high();
                    }
                    periph.mosi.set_low();

                    periph.nlp_disa.set_low();
                    (
                        Some(result),
                        Self::Idle {
                            periph,
                            buf: buf.release(),
                            invert_idle,
                            invert_data,
                            invert_sck_pol,
                        },
                    )
                } else {
                    (None, s)
                }
            },
        )
    }
}

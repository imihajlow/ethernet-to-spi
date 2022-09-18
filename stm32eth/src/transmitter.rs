use replace_with::{replace_with_and_return};
use stm32f1xx_hal::{
    afio::MAPR,
    device::SPI1,
    dma::dma1,
    gpio::{Cr, Output, PA5, PA7, PB1},
    spi::{Spi, NoMiso, Mode, Polarity, Phase, SpiBitFormat},
    rcc::Clocks,
    prelude::*,
};

use crate::tx_frame_buf::TxFrameBuf;

pub const BUFFER_LEN: usize = 1600;

pub type TxBuf = &'static mut [u8; BUFFER_LEN];

pub struct SpiTxPeriph {
    spi: SPI1,
    sck: PA5<Output>,
    mosi: PA7<Output>,
    nlp_disa: PB1<Output>,
    dma: dma1::C3,
    cr: Cr<'A', false>,
    mapr: MAPR,
    clocks: Clocks,
}

pub enum Transmitter {
    Idle(SpiTxPeriph, TxBuf),
    Invalid,
}

impl Transmitter {
    pub fn new(
        spi: SPI1,
        mut sck: PA5<Output>,
        mut mosi: PA7<Output>,
        mut nlp_disa: PB1<Output>,
        dma: dma1::C3,
        cr: Cr<'A', false>,
        mapr: MAPR,
        clocks: Clocks,
        buf: TxBuf
    ) -> Self {
        nlp_disa.set_low();
        sck.set_high();
        mosi.set_low();
        Self::Idle(
            SpiTxPeriph {
                spi,
                sck,
                mosi,
                nlp_disa,
                dma,
                cr,
                mapr,
                clocks,
            },
            buf,
        )
    }

    pub fn transmit<R, F>(&mut self, len: usize, f: F) -> Option<R>
    where
        F: FnOnce(&mut [u8]) -> R,
    {
        replace_with_and_return(self, || Self::Invalid, |s| {
            if let Self::Idle(mut periph, buf) = s {
                let (frame_buf, result) = TxFrameBuf::new_with_fn(buf, len, f);

                periph.nlp_disa.set_high();

                let sck_alt = periph.sck.into_alternate_push_pull(&mut periph.cr);
                let mosi_alt = periph.mosi.into_alternate_push_pull(&mut periph.cr);
                let mut spi1 = Spi::spi1(
                    periph.spi,
                    (sck_alt, NoMiso, mosi_alt),
                    &mut periph.mapr,
                    Mode {
                        polarity: Polarity::IdleLow,
                        phase: Phase::CaptureOnFirstTransition,
                    },
                    10.MHz(),
                    periph.clocks,
                );
                spi1.bit_format(SpiBitFormat::LsbFirst);
                let spi_dma = spi1.with_tx_dma(periph.dma);

                let transfer = spi_dma.write(frame_buf);

                let (frame_buf, spi_dma) = transfer.wait();

                let buf = frame_buf.release();

                let (spi1, dma) = spi_dma.release();
                periph.dma = dma;

                while spi1.is_busy() {}

                let (spi, (sck_alt, _, mosi_alt)) = spi1.release();
                periph.spi = spi;
                periph.sck = sck_alt.into_push_pull_output(&mut periph.cr);
                periph.mosi = mosi_alt.into_push_pull_output(&mut periph.cr);
                periph.sck.set_high();
                periph.mosi.set_low();

                periph.nlp_disa.set_low();
                (Some(result), Self::Idle(periph, buf))
            } else {
                (None, s)
            }
        })
    }
}

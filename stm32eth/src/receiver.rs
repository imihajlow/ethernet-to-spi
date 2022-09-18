use crc::{Crc, CRC_32_ISO_HDLC};
use stm32f1xx_hal::{
    device::SPI2,
    dma::{self, dma1, ReadDma, RxDma},
    gpio::{ExtiPin, PB13, PB15, PC6},
    spi::{Mode, NoMiso, Phase, Polarity, Slave, Spi, Spi2NoRemap, SpiBitFormat},
};
use core::fmt;

use replace_with::{replace_with, replace_with_and_return};

pub const BUFFER_LEN: usize = 1600;
const MIN_FRAME_LEN: usize = 8 + 2 * 6 + 2 + 4;
const PREAMBLE: [u8; 8] = [0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0xd5];
const CRC: Crc<u32> = Crc::<u32>::new(&CRC_32_ISO_HDLC);

type PinCs = PC6;
type PinSck = PB13;
type PinMosi = PB15;

pub struct SpiRxPeriph {
    spi: SPI2,
    pin_sck: PinSck,
    pin_mosi: PinMosi,
    pin_cs: PinCs,
    dma: dma1::C4,
}

pub type RxBuf = &'static mut [u8; BUFFER_LEN];

type Transfer = dma::Transfer<
    dma::W,
    RxBuf,
    RxDma<Spi<SPI2, Spi2NoRemap, (PinSck, NoMiso, PinMosi), u8, Slave>, dma1::C4>,
>;

pub enum Receiver {
    Idle(SpiRxPeriph, RxBuf),
    WaitFrameEnd(SpiRxPeriph, RxBuf),
    Listen(Transfer, PinCs),
    Data(SpiRxPeriph, RxBuf, usize),
    Process(SpiRxPeriph),
    Invalid,
}

pub enum FrameError {
    InvalidLength(usize),
    InvalidFcs,
    InvalidPreamble,
    InvalidState,
    StartListen,
}

impl Receiver {
    pub fn new(
        spi: SPI2,
        pin_sck: PinSck,
        pin_mosi: PinMosi,
        pin_cs: PinCs,
        dma: dma1::C4,
        buf: RxBuf,
    ) -> Self {
        Self::Idle(
            SpiRxPeriph {
                spi,
                pin_sck,
                pin_mosi,
                pin_cs,
                dma,
            },
            buf,
        )
    }

    pub fn try_get_data(&mut self) -> Option<(RxBuf, usize)> {
        replace_with_and_return(
            self,
            || Self::Invalid,
            |s| match s {
                Self::Data(periph, buf, len) => (Some((buf, len)), Self::Process(periph)),
                _ => (None, s),
            },
        )
    }

    pub fn return_buffer(&mut self, buf: RxBuf) {
        replace_with(
            self,
            || Self::Invalid,
            |s| {
                if let Self::Process(periph) = s {
                    Self::Idle(periph, buf)
                } else {
                    Self::Invalid
                }
            },
        );
    }

    pub fn start_listen(&mut self) {
        replace_with(
            self,
            || Self::Invalid,
            |s| {
                match s {
                    Self::Idle(periph, buf) => {
                        if periph.pin_cs.is_high() {
                            // a frame is already being sent, we've missed it
                            Self::WaitFrameEnd(periph, buf)
                        } else {
                            let (transfer, pin_cs) = create_dma_transfer(periph, buf);
                            Self::Listen(transfer, pin_cs)
                        }
                    }
                    Self::WaitFrameEnd(periph, buf) => {
                        let (transfer, pin_cs) = create_dma_transfer(periph, buf);
                        Self::Listen(transfer, pin_cs)
                    }
                    _ => Self::Invalid,
                }
            },
        )
    }

    pub fn on_frame_end(&mut self) -> Result<usize, FrameError> {
        if let Self::WaitFrameEnd(_, _) = self {
            self.start_listen();
            Err(FrameError::StartListen)
        } else {
            let result = replace_with_and_return(
                self,
                || Self::Invalid,
                |s| {
                    match s {
                        Self::Listen(transfer, pin_cs) => {
                            let (ret_buf, ret_rx) = transfer.stop();
                            let ndtr = ret_rx.channel.get_ndtr() as usize;
                            let len = ret_buf.len() - ndtr;
                            let result = if len < MIN_FRAME_LEN {
                                Err(FrameError::InvalidLength(len))
                            } else {
                                if ret_buf[0..8] != PREAMBLE {
                                    Err(FrameError::InvalidPreamble)
                                } else {
                                    let mut digest = CRC.digest();
                                    digest.update(&ret_buf[8..len - 4]);
                                    let fcs = digest.finalize().to_le_bytes();
                                    if ret_buf[len - 4..len] == fcs {
                                        Ok(len)
                                    } else {
                                        Err(FrameError::InvalidFcs)
                                    }
                                }
                            };

                            // release SPI to reset it (in case there's an incomplete byte in the register)
                            let (spi, dma) = ret_rx.release();
                            let (spi, (pin_sck, _, pin_mosi)) = spi.release();
                            let periph = SpiRxPeriph {
                                spi,
                                pin_sck,
                                pin_mosi,
                                dma,
                                pin_cs,
                            };
                            if result.is_ok() {
                                (result, Self::Data(periph, ret_buf, len))
                            } else {
                                (result, Self::Idle(periph, ret_buf))
                            }
                        }
                        _ => (Err(FrameError::InvalidState), s),
                    }
                },
            );
            if let Self::Idle(_, _) = self {
                self.start_listen()
            }
            result
        }
    }

    pub fn clear_cs_interrupt(&mut self) {
        match self {
            Self::Idle(periph, _)
            | Self::WaitFrameEnd(periph, _)
            | Self::Data(periph, _, _)
            | Self::Process(periph) => {
                periph.pin_cs.clear_interrupt_pending_bit();
            }
            Self::Listen(_, pin_cs) => {
                pin_cs.clear_interrupt_pending_bit();
            }
            Self::Invalid => (),
        };
    }
}

fn create_dma_transfer(periph: SpiRxPeriph, buf: RxBuf) -> (Transfer, PinCs) {
    const MODE: Mode = Mode {
        phase: Phase::CaptureOnFirstTransition,
        polarity: Polarity::IdleLow,
    };
    let mut spi2 = Spi::spi2_slave(periph.spi, (periph.pin_sck, NoMiso, periph.pin_mosi), MODE);
    spi2.bit_format(SpiBitFormat::LsbFirst);
    let spi_dma_rx = spi2.with_rx_dma(periph.dma);

    let transfer = spi_dma_rx.read(buf);

    (transfer, periph.pin_cs)
}

impl fmt::Display for Receiver {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Idle(_, _) => f.write_str("Idle"),
            Self::WaitFrameEnd(_, _) => f.write_str("WaitFrameEnd"),
            Self::Listen(_, _) => f.write_str("Listen"),
            Self::Data(_, _, _) => f.write_str("Data"),
            Self::Process(_) => f.write_str("Process"),
            Self::Invalid => f.write_str("Invalid"),
        }
    }
}

impl fmt::Display for FrameError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            FrameError::InvalidLength(l) => write!(f, "invalid length: {}", l),
            FrameError::InvalidFcs => f.write_str("invalid FCS"),
            FrameError::InvalidState => f.write_str("invalid state"),
            FrameError::InvalidPreamble => f.write_str("wrong preamble"),
            FrameError::StartListen => f.write_str("last frame dropped"),
        }
    }
}

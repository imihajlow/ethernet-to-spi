use core::fmt;
use crc::{Crc, CRC_32_ISO_HDLC};
use defmt::Format;
use stm32f4xx_hal::{
    dma::{self, config::DmaConfig, traits::Stream},
    gpio::{ExtiPin, NoPin, PB13, PB15, PC6},
    pac::{DMA1, SPI2},
    prelude::*,
    rcc::Clocks,
    spi::{self, BitFormat, Mode, NoMiso, Phase, Polarity, Slave, Spi},
};

use replace_with::{replace_with, replace_with_and_return};

pub const BUFFER_LEN: usize = 1600;
const MIN_FRAME_LEN: usize = 2 * 6 + 2 + 4;
const CRC: Crc<u32> = Crc::<u32>::new(&CRC_32_ISO_HDLC);

type PinCs = PC6;
type PinSck = PB13;
type PinMosi = PB15;

type DmaStream = dma::Stream3<DMA1>;

pub struct SpiRxPeriph {
    spi: Spi<SPI2, (PinSck, NoPin, PinMosi), false, u8, Slave>,
    pin_cs: PinCs,
    dma_stream: DmaStream,
}

pub type RxBuf = &'static mut [u8; BUFFER_LEN];

type Transfer = dma::Transfer<
    dma::Stream3<DMA1>,
    0,
    spi::Rx<SPI2, (PinSck, NoMiso, PinMosi), false, Slave>,
    dma::PeripheralToMemory,
    RxBuf,
>;

pub enum Receiver {
    Idle(SpiRxPeriph, RxBuf),
    WaitFrameEnd(SpiRxPeriph, RxBuf),
    Listen(Transfer, PinCs),
    Data(SpiRxPeriph, RxBuf, usize),
    Process(SpiRxPeriph),
    Invalid,
}

#[derive(Format)]
pub enum FrameError {
    InvalidLength(usize),
    InvalidFcs,
    InvalidPreamble,
    InvalidState(&'static str),
    StartListen,
}

impl Receiver {
    pub fn new(
        spi: SPI2,
        pin_sck: PinSck,
        pin_mosi: PinMosi,
        pin_cs: PinCs,
        dma_stream: dma::Stream3<DMA1>,
        buf: RxBuf,
        clocks: &Clocks,
    ) -> Self {
        let mut spi = Spi::new_slave(
            spi,
            (pin_sck, NoMiso {}, pin_mosi),
            Mode {
                polarity: Polarity::IdleLow,
                phase: Phase::CaptureOnFirstTransition,
            },
            10_000_000.Hz(),
            &clocks,
        );
        spi.bit_format(BitFormat::LsbFirst);
        spi.set_internal_nss(true);
        Self::Idle(
            SpiRxPeriph {
                spi,
                pin_cs,
                dma_stream,
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
        self.start_listen();
    }

    pub fn start_listen(&mut self) {
        replace_with(
            self,
            || Self::Invalid,
            |s| {
                match s {
                    Self::Idle(periph, buf) => {
                        if periph.pin_cs.is_low() {
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
                            let (stream, ret_rx, mut ret_buf, _) = transfer.release();
                            let mut spi = ret_rx.release();
                            spi.set_internal_nss(true);
                            let stream: DmaStream = stream; // ensure type
                            let ndtr = DmaStream::get_number_of_transfers() as usize;
                            let len = ret_buf.len() - ndtr;
                            defmt::info!("got {:X}", ret_buf[0..len]);
                            let result = if len < MIN_FRAME_LEN {
                                Err(FrameError::InvalidLength(len))
                            } else {
                                // first SCK edge comes after MOSI/Manchester transition
                                ret_buf[0] = ret_buf[0] ^ 0x01;
                                let mut digest = CRC.digest();
                                digest.update(&ret_buf[0..len - 4]);
                                let fcs = digest.finalize().to_le_bytes();
                                if ret_buf[len - 4..len] == fcs {
                                    Ok(len)
                                } else {
                                    Err(FrameError::InvalidFcs)
                                }
                            };

                            // release SPI to reset it (in case there's an incomplete byte in the register)
                            let periph = SpiRxPeriph {
                                spi,
                                dma_stream: stream,
                                pin_cs,
                            };
                            if result.is_ok() {
                                (result, Self::Data(periph, ret_buf, len))
                            } else {
                                (result, Self::Idle(periph, ret_buf))
                            }
                        }
                        _ => (Err(FrameError::InvalidState(s.get_state_name())), s),
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

    fn get_state_name(&self) -> &'static str {
        match self {
            Receiver::Idle(_, _) => "Idle",
            Receiver::WaitFrameEnd(_, _) => "WaitFrameEnd",
            Receiver::Listen(_, _) => "Listen",
            Receiver::Data(_, _, _) => "Data",
            Receiver::Process(_) => "Process",
            Receiver::Invalid => "Invalid",
        }
    }
}

fn create_dma_transfer(periph: SpiRxPeriph, buf: RxBuf) -> (Transfer, PinCs) {
    let mut spi = periph.spi;
    spi.set_internal_nss(false);
    let rx = spi.use_dma().rx();
    let mut transfer = Transfer::init_peripheral_to_memory(
        periph.dma_stream,
        rx,
        buf,
        None,
        DmaConfig::default().memory_increment(true),
    );
    transfer.start(|_rx| {});

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
            FrameError::InvalidState(s) => write!(f, "invalid state {}", s),
            FrameError::InvalidPreamble => f.write_str("wrong preamble"),
            FrameError::StartListen => f.write_str("last frame dropped"),
        }
    }
}

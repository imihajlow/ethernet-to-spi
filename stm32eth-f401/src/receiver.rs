use crc::{Crc, CRC_32_ISO_HDLC};
use defmt::Format;
use heapless::spsc::Queue;
use cortex_m::asm;
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
pub const MAX_BUFFERS: usize = 4;
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

pub struct Receiver {
    vacant_buffers: Queue<RxBuf, MAX_BUFFERS>,
    received_frames: Queue<(RxBuf, usize), MAX_BUFFERS>,
    state: ReceiverState,
    stats: Stats,
}

enum ReceiverState {
    Idle(SpiRxPeriph),
    Listen(Transfer, PinCs),
}

#[derive(Default)]
struct Stats {
    flipped: usize,
    missed: usize,
    received: usize,
    bad_fcs: usize,
    iteration: usize,
}

#[derive(Format)]
pub enum FrameError {
    InvalidLength(usize),
    InvalidFcs,
    InvalidPreamble,
    Missed,
}

impl Receiver {
    pub fn new(
        spi: SPI2,
        pin_sck: PinSck,
        pin_mosi: PinMosi,
        pin_cs: PinCs,
        dma_stream: dma::Stream3<DMA1>,
        buffers: [RxBuf; MAX_BUFFERS],
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
        let mut vacant_buffers = Queue::new();
        for buf in buffers.into_iter() {
            vacant_buffers.enqueue(buf).ok();
        }
        Self {
            vacant_buffers,
            received_frames: Queue::new(),
            state: ReceiverState::Idle(SpiRxPeriph {
                spi,
                pin_cs,
                dma_stream,
            }),
            stats: Default::default(),
        }
    }

    pub fn try_get_data(&mut self) -> Option<(RxBuf, usize)> {
        self.received_frames.dequeue()
    }

    pub fn return_buffer(&mut self, buf: RxBuf) {
        self.vacant_buffers.enqueue(buf).ok();
        self.start_listen();
    }

    pub fn start_listen(&mut self) {
        replace_with(
            &mut self.state,
            || panic!(),
            |state| match state {
                ReceiverState::Idle(periph) => {
                    if periph.pin_cs.is_low() {
                        // a frame is already being sent, we've missed it
                        ReceiverState::Idle(periph)
                    } else {
                        if let Some(buf) = self.vacant_buffers.dequeue() {
                            let (transfer, pin_cs) = create_dma_transfer(periph, buf);
                            ReceiverState::Listen(transfer, pin_cs)
                        } else {
                            ReceiverState::Idle(periph)
                        }
                    }
                }
                s => s,
            },
        );
    }

    pub fn on_edge_maybe(&mut self) -> Option<Result<usize, FrameError>> {
        let is_low = self.clear_cs_interrupt_and_return_true_if_still_low();
        if is_low {
            defmt::info!("still low");
            return None;
        }
        let result = replace_with_and_return(
            &mut self.state,
            || panic!(),
            |state| match state {
                ReceiverState::Listen(transfer, pin_cs) => {
                    let (stream, ret_rx, mut ret_buf, _) = transfer.release();
                    let mut spi = ret_rx.release();
                    spi.set_internal_nss(true);
                    let stream: DmaStream = stream; // ensure type
                    let ndtr = DmaStream::get_number_of_transfers() as usize;
                    let len = ret_buf.len() - ndtr;
                    defmt::info!("got {:02X}", ret_buf[0..len]);
                    let result = if len < MIN_FRAME_LEN {
                        Err(FrameError::InvalidLength(len))
                    } else {
                        let mut digest = CRC.digest();
                        digest.update(&ret_buf[0..len - 4]);
                        let fcs = digest.finalize().to_le_bytes();
                        if ret_buf[len - 4..len] == fcs {
                            Ok(len)
                        } else {
                            // try flipping the first bit
                            // first SCK edge comes after MOSI/Manchester transition
                            self.stats.count_flip();
                            ret_buf[0] = ret_buf[0] ^ 0x01;
                            let mut digest = CRC.digest();
                            digest.update(&ret_buf[0..len - 4]);
                            let fcs = digest.finalize().to_le_bytes();
                            if ret_buf[len - 4..len] == fcs {
                                Ok(len)
                            } else {
                                Err(FrameError::InvalidFcs)
                            }
                        }
                    };

                    // release SPI to reset it (in case there's an incomplete byte in the register)
                    let periph = SpiRxPeriph {
                        spi,
                        dma_stream: stream,
                        pin_cs,
                    };

                    if result.is_ok() {
                        self.stats.count_good();
                        self.received_frames.enqueue((ret_buf, len)).ok();
                    } else {
                        self.stats.count_bad_fcs();
                        self.vacant_buffers.enqueue(ret_buf).ok();
                    }
                    (Some(result), ReceiverState::Idle(periph))
                }
                ReceiverState::Idle(_) => {
                    self.stats.count_miss();
                    (Some(Err(FrameError::Missed)), state)
                }
            },
        );
        if let ReceiverState::Idle(_) = self.state {
            self.start_listen();
        }
        self.stats.dump();
        result
    }

    fn clear_cs_interrupt_and_return_true_if_still_low(&mut self) -> bool {
        asm::delay(150);
        match &mut self.state {
            ReceiverState::Idle(periph) => {
                periph.pin_cs.clear_interrupt_pending_bit();
                periph.pin_cs.is_low()
            }
            ReceiverState::Listen(_, pin_cs) => {
                pin_cs.clear_interrupt_pending_bit();
                pin_cs.is_low()
            }
        }
    }
}

impl Stats {
    fn count_miss(&mut self) {
        self.missed += 1;
    }

    fn count_flip(&mut self) {
        self.flipped += 1;
    }

    fn count_good(&mut self) {
        self.received += 1;
    }

    fn count_bad_fcs(&mut self) {
        self.bad_fcs += 1;
    }

    fn dump(&mut self) {
        self.iteration += 1;
        if self.iteration == 10 {
            defmt::info!(
                "Good: {}, bad FCS: {}, missed: {}, flipped: {}",
                self.received,
                self.bad_fcs,
                self.missed,
                self.flipped
            );
            self.iteration = 0;
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

use crc::{Crc, CRC_32_ISO_HDLC};
use embedded_dma::ReadBuffer;

pub struct TxFrameBuf<const L: usize> {
    buf: &'static mut [u8; L],
    len: usize,
}

const PREAMBLE: [u8; 8] = [0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0xd5];

const CRC: Crc<u32> = Crc::<u32>::new(&CRC_32_ISO_HDLC);

impl<const L: usize> TxFrameBuf<L> {
    pub fn new(buf: &'static mut [u8; L], data: &[u8]) -> Self {
        let len = data.len();
        buf[0..PREAMBLE.len()].copy_from_slice(&PREAMBLE);
        buf[PREAMBLE.len()..len + PREAMBLE.len()].copy_from_slice(data);
        let mut digest = CRC.digest();
        digest.update(data);
        let crc = digest.finalize().to_le_bytes();
        buf[len + PREAMBLE.len()..len + PREAMBLE.len() + crc.len()].copy_from_slice(&crc);
        Self {
            buf: buf,
            len: data.len() + PREAMBLE.len() + crc.len(),
        }
    }

    pub fn release(self) -> &'static mut [u8; L] {
        self.buf
    }
}

unsafe impl<const L: usize> ReadBuffer for TxFrameBuf<L> {
    type Word = u8;

    unsafe fn read_buffer(&self) -> (*const Self::Word, usize) {
        (self.buf.as_ptr(), self.len)
    }
}
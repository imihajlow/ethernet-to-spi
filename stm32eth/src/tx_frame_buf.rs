use core::cmp::max;
use crc::{Crc, CRC_32_ISO_HDLC};
use embedded_dma::ReadBuffer;

pub struct TxFrameBuf<const L: usize> {
    buf: &'static mut [u8; L],
    len: usize,
}

const PREAMBLE: [u8; 8] = [0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0xd5];

const CRC: Crc<u32> = Crc::<u32>::new(&CRC_32_ISO_HDLC);

impl<const L: usize> TxFrameBuf<L> {
    pub fn new_with_fn<F, R>(buf: &'static mut [u8; L], len: usize, f: F) -> (Self, R)
    where
        F: FnOnce(&mut [u8]) -> R,
    {
        buf[0..PREAMBLE.len()].copy_from_slice(&PREAMBLE);
        let slice = &mut buf[PREAMBLE.len()..PREAMBLE.len()+len];
        let result = f(slice);
        let padded_len = max(len, 60);
        buf[PREAMBLE.len()+len..PREAMBLE.len()+padded_len].fill(0);
        let mut digest = CRC.digest();
        digest.update(&buf[PREAMBLE.len()..PREAMBLE.len()+padded_len]);
        let crc = digest.finalize().to_le_bytes();
        buf[padded_len + PREAMBLE.len()..padded_len + PREAMBLE.len() + crc.len()].copy_from_slice(&crc);
        (
            Self {
                buf: buf,
                len: padded_len + PREAMBLE.len() + crc.len(),
            },
            result,
        )
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

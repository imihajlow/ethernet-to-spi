use crate::receiver::{Receiver, RxBuf};
use cortex_m::singleton;
use smoltcp::phy::Device;

pub struct SpiDevice<MR>
where
    MR: rtic::Mutex<T = Receiver>,
{
    receiver: MR,
    rx_buf: Option<RxBuf>,
    tx_buf: &'static mut [u8; 1600],
}

impl<MR: rtic::Mutex<T = Receiver>> SpiDevice<MR> {
    pub fn new(receiver: MR) -> Option<Self> {
        Some(Self {
            receiver,
            rx_buf: None,
            tx_buf: singleton!(: [u8; 1600] = [0; 1600])?,
        })
    }
}

impl<'a, MR: rtic::Mutex<T = Receiver> + 'a> Device<'a> for SpiDevice<MR> {
    type RxToken = RxToken<'a, MR>;

    type TxToken = TxToken;

    fn receive(&'a mut self) -> Option<(Self::RxToken, Self::TxToken)> {
        let b = self.receiver.lock(|receiver| receiver.try_get_data());
        match b {
            Some((buf, len)) => Some((
                RxToken {
                    buf,
                    len,
                    receiver: &mut self.receiver,
                },
                TxToken(),
            )),
            None => None,
        }
    }

    fn transmit(&'a mut self) -> Option<Self::TxToken> {
        todo!()
    }

    fn capabilities(&self) -> smoltcp::phy::DeviceCapabilities {
        todo!()
    }
}

pub struct RxToken<'a, MR: rtic::Mutex<T = Receiver>> {
    buf: RxBuf,
    len: usize,
    receiver: &'a mut MR,
}

pub struct TxToken();

impl<'a, MR: rtic::Mutex<T = Receiver>> smoltcp::phy::RxToken for RxToken<'a, MR> {
    fn consume<R, F>(self, timestamp: smoltcp::time::Instant, f: F) -> smoltcp::Result<R>
    where
        F: FnOnce(&mut [u8]) -> smoltcp::Result<R>,
    {
        let result = f(&mut self.buf[8..self.len - 4]);
        self.receiver.lock(|receiver| {
            receiver.return_buffer(self.buf);
        });
        result
    }
}

impl smoltcp::phy::TxToken for TxToken {
    fn consume<R, F>(
        self,
        timestamp: smoltcp::time::Instant,
        len: usize,
        f: F,
    ) -> smoltcp::Result<R>
    where
        F: FnOnce(&mut [u8]) -> smoltcp::Result<R>,
    {
        todo!()
    }
}

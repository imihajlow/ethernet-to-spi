use crate::{
    receiver::{Receiver, RxBuf},
    transmitter::Transmitter,
};

use core::cell::RefCell;
use cortex_m::interrupt;
use smoltcp::phy::{Device, DeviceCapabilities, Medium};

pub type ReceiverMutex = &'static cortex_m::interrupt::Mutex<RefCell<Option<Receiver>>>;
pub struct SpiDevice {
    receiver: ReceiverMutex,
    transmitter: Transmitter,
}

impl SpiDevice {
    pub fn new(receiver: ReceiverMutex, transmitter: Transmitter) -> Self {
        Self {
            receiver,
            transmitter,
        }
    }
}

impl<'a> Device<'a> for SpiDevice {
    type RxToken = RxToken;

    type TxToken = TxToken<'a>;

    fn receive(&'a mut self) -> Option<(Self::RxToken, Self::TxToken)> {
        let b = interrupt::free(|cs| {
            self.receiver
                .borrow(cs)
                .borrow_mut()
                .as_mut()
                .unwrap()
                .try_get_data()
        });
        match b {
            Some((buf, len)) => Some((
                RxToken {
                    buf,
                    len,
                    receiver: self.receiver,
                },
                TxToken(&mut self.transmitter),
            )),
            None => None,
        }
    }

    fn transmit(&'a mut self) -> Option<Self::TxToken> {
        Some(TxToken(&mut self.transmitter))
    }

    fn capabilities(&self) -> smoltcp::phy::DeviceCapabilities {
        let mut caps = DeviceCapabilities::default();
        caps.medium = Medium::Ethernet;
        caps.max_transmission_unit = crate::transmitter::MTU;
        caps.max_burst_size = Some(1);
        caps
    }
}

pub struct RxToken {
    buf: RxBuf,
    len: usize,
    receiver: ReceiverMutex,
}

pub struct TxToken<'a>(&'a mut Transmitter);

impl smoltcp::phy::RxToken for RxToken {
    fn consume<R, F>(self, _timestamp: smoltcp::time::Instant, f: F) -> smoltcp::Result<R>
    where
        F: FnOnce(&mut [u8]) -> smoltcp::Result<R>,
    {
        let result = f(&mut self.buf[8..self.len - 4]);
        interrupt::free(|cs| {
            self.receiver
                .borrow(cs)
                .borrow_mut()
                .as_mut()
                .unwrap()
                .return_buffer(self.buf)
        });
        result
    }
}

impl<'a> smoltcp::phy::TxToken for TxToken<'a> {
    fn consume<R, F>(
        self,
        _timestamp: smoltcp::time::Instant,
        len: usize,
        f: F,
    ) -> smoltcp::Result<R>
    where
        F: FnOnce(&mut [u8]) -> smoltcp::Result<R>,
    {
        self.0.transmit(len, f).unwrap()
    }
}

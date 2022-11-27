use core::future::Future;
use core::task::Poll;
use smoltcp::time::Instant;

use embedded_io::asynch::{Read, Write};
use embedded_io::Io;
use smoltcp::iface::{Interface, SocketHandle};
use smoltcp::socket::TcpSocket;

use crate::device::SpiDevice;

type GetTicks = fn() -> i64;

pub struct TcpSocketAdapter<'a> {
    iface: Interface<'a, SpiDevice>,
    handle: SocketHandle,
    get_ticks: GetTicks,
}

#[derive(Debug)]
pub struct TcpSocketAdapterError(smoltcp::Error);

pub struct ReadFuture<'a, 'b>
where
    'a: 'b,
{
    adapter: &'b mut TcpSocketAdapter<'a>,
    buf: &'b mut [u8],
}

pub struct WriteFuture<'a, 'b>
where
    'a: 'b,
{
    adapter: &'b mut TcpSocketAdapter<'a>,
    buf: &'b [u8],
}

pub struct FlushFuture();

impl<'a> TcpSocketAdapter<'a> {
    pub fn new(
        iface: Interface<'a, SpiDevice>,
        handle: SocketHandle,
        get_ticks: GetTicks,
    ) -> Self {
        Self {
            iface,
            handle,
            get_ticks,
        }
    }

    pub fn release(self) -> Interface<'a, SpiDevice> {
        self.iface
    }
}

impl From<smoltcp::Error> for TcpSocketAdapterError {
    fn from(e: smoltcp::Error) -> Self {
        Self(e)
    }
}

impl embedded_io::Error for TcpSocketAdapterError {
    fn kind(&self) -> embedded_io::ErrorKind {
        embedded_io::ErrorKind::Other
    }
}

impl<'a> Io for TcpSocketAdapter<'a> {
    type Error = TcpSocketAdapterError;
}

impl<'a> Read for TcpSocketAdapter<'a> {
    type ReadFuture<'c> = ReadFuture<'a, 'c> where Self: 'c;

    fn read<'c>(&'c mut self, buf: &'c mut [u8]) -> Self::ReadFuture<'c> {
        ReadFuture { adapter: self, buf }
    }
}

impl<'a> Write for TcpSocketAdapter<'a> {
    type WriteFuture<'c> = WriteFuture<'a, 'c> where Self: 'c;
    type FlushFuture<'c> = FlushFuture where Self: 'c;

    fn write<'c>(&'c mut self, buf: &'c [u8]) -> Self::WriteFuture<'c> {
        WriteFuture { adapter: self, buf }
    }

    fn flush<'c>(&'c mut self) -> Self::FlushFuture<'c> {
        FlushFuture()
    }
}

impl<'a, 'b> ReadFuture<'a, 'b> {
    fn recv(&mut self) -> Result<usize, TcpSocketAdapterError> {
        let sock = self
            .adapter
            .iface
            .get_socket::<TcpSocket>(self.adapter.handle);
        sock.recv_slice(self.buf).map_err(Into::into)
    }

    fn can_recv(&mut self) -> Result<bool, TcpSocketAdapterError> {
        let now = Instant::from_millis((self.adapter.get_ticks)());
        self.adapter.iface.poll(now)?;
        let sock = self
            .adapter
            .iface
            .get_socket::<TcpSocket>(self.adapter.handle);
        Ok(sock.can_recv())
    }
}

impl<'a, 'b> Future for ReadFuture<'a, 'b> {
    type Output = Result<usize, TcpSocketAdapterError>;
    fn poll(
        mut self: core::pin::Pin<&mut Self>,
        ctx: &mut core::task::Context<'_>,
    ) -> Poll<<Self as Future>::Output> {
        match (*self).can_recv() {
            Ok(true) => Poll::Ready((*self).recv()),
            Ok(false) => {
                ctx.waker().wake_by_ref();
                Poll::Pending
            }
            Err(e) => Poll::Ready(Err(e)),
        }
    }
}

impl<'a, 'b> WriteFuture<'a, 'b> {
    fn send(&mut self) -> Result<usize, TcpSocketAdapterError> {
        let sock = self
            .adapter
            .iface
            .get_socket::<TcpSocket>(self.adapter.handle);
        sock.send_slice(self.buf).map_err(Into::into)
    }

    fn can_send(&mut self) -> Result<bool, TcpSocketAdapterError> {
        let now = Instant::from_millis((self.adapter.get_ticks)());
        self.adapter.iface.poll(now)?;
        let sock = self
            .adapter
            .iface
            .get_socket::<TcpSocket>(self.adapter.handle);
        Ok(sock.can_send())
    }
}

impl<'a, 'b> Future for WriteFuture<'a, 'b> {
    type Output = Result<usize, TcpSocketAdapterError>;
    fn poll(
        mut self: core::pin::Pin<&mut Self>,
        ctx: &mut core::task::Context<'_>,
    ) -> Poll<<Self as Future>::Output> {
        match (*self).can_send() {
            Ok(true) => Poll::Ready((*self).send()),
            Ok(false) => {
                ctx.waker().wake_by_ref();
                Poll::Pending
            }
            Err(e) => Poll::Ready(Err(e)),
        }
    }
}

impl Future for FlushFuture {
    type Output = Result<(), TcpSocketAdapterError>;
    fn poll(
        self: core::pin::Pin<&mut Self>,
        _: &mut core::task::Context<'_>,
    ) -> Poll<<Self as Future>::Output> {
        Poll::Ready(Ok(()))
    }
}

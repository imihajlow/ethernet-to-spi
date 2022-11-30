use core::future::Future;
use core::task::Poll;
use smoltcp::time::{Instant, Duration};

use embedded_io::asynch::{Read, Write};
use embedded_io::Io;
use smoltcp::iface::{Interface, SocketHandle};
use smoltcp::socket::TcpSocket;
use smoltcp::wire::IpEndpoint;

use crate::device::SpiDevice;

type GetTicks = fn() -> i64;

pub struct TcpSocketAdapter<'a> {
    iface: Interface<'a, SpiDevice>,
    handle: SocketHandle,
    pub get_ticks: GetTicks,
}

#[derive(Debug, defmt::Format)]
pub enum TcpSocketAdapterError {
    Smoltcp(smoltcp::Error),
    Timeout
}

pub struct ReadFuture<'socket, 'adapter, 'buf>
where
    'socket: 'adapter,
    'socket: 'buf
{
    adapter: &'adapter mut TcpSocketAdapter<'socket>,
    buf: &'buf mut [u8],
}

pub struct WriteFuture<'socket, 'adapter, 'buf>
where
    'socket: 'adapter,
    'socket: 'buf
{
    adapter: &'adapter mut TcpSocketAdapter<'socket>,
    buf: &'buf [u8],
}

pub struct ConnectFuture<'a, 'b>
where
    'a: 'b,
{
    adapter: &'b mut TcpSocketAdapter<'a>,
    remote_endpoint: IpEndpoint,
    local_endpoint: IpEndpoint,
    connecting: bool,
}

pub struct CloseFuture<'a, 'b>
where
    'a: 'b,
{
    adapter: &'b mut TcpSocketAdapter<'a>,
}

pub struct FlushFuture();

impl<'a> TcpSocketAdapter<'a> {
    pub fn new(iface: Interface<'a, SpiDevice>, handle: SocketHandle, get_ticks: GetTicks) -> Self {
        Self {
            iface,
            handle,
            get_ticks,
        }
    }

    pub fn release(self) -> Interface<'a, SpiDevice> {
        self.iface
    }

    pub fn connect<'b, T: Into<IpEndpoint>, U: Into<IpEndpoint>>(
        &'b mut self,
        remote_endpoint: T,
        local_endpoint: U,
    ) -> ConnectFuture<'a, 'b> {
        let time_start = (self.get_ticks)();
        ConnectFuture {
            adapter: self,
            remote_endpoint: remote_endpoint.into(),
            local_endpoint: local_endpoint.into(),
            connecting: false,
        }
    }

    pub fn close<'b>(&'b mut self) -> CloseFuture<'a, 'b> {
        {
            let sock = self.iface.get_socket::<TcpSocket>(self.handle);
            sock.close();
        }
        CloseFuture { adapter: self }
    }

    fn poll(&mut self) -> Result<(), TcpSocketAdapterError> {
        let now = Instant::from_millis((self.get_ticks)());
        self.iface.poll(now)?;
        Ok(())
    }

    fn is_active(&mut self) -> bool {
        let sock = self.iface.get_socket::<TcpSocket>(self.handle);
        sock.is_active()
    }

    fn can_send(&mut self) -> bool {
        let sock = self.iface.get_socket::<TcpSocket>(self.handle);
        sock.can_send()
    }

    fn can_recv(&mut self) -> bool {
        let sock = self.iface.get_socket::<TcpSocket>(self.handle);
        sock.can_recv()
    }
}

impl From<smoltcp::Error> for TcpSocketAdapterError {
    fn from(e: smoltcp::Error) -> Self {
        Self::Smoltcp(e)
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

impl<'socket> Read for TcpSocketAdapter<'socket> {
    // type ReadFuture<'c> = ReadFuture<'a, 'c> where Self: 'c;

    fn read<'adapter, 'buf>(&'adapter mut self, buf: &'buf mut [u8]) -> ReadFuture<'socket, 'adapter, 'buf> {
        let time_start = (self.get_ticks)();
        ReadFuture {
            adapter: self,
            buf,
        }
    }
}

impl<'socket> Write for TcpSocketAdapter<'socket> {
    fn write<'adapter, 'buf>(&'adapter mut self, buf: &'buf [u8]) -> WriteFuture<'socket, 'adapter, 'buf> {
        let time_start = (self.get_ticks)();
        WriteFuture {
            adapter: self,
            buf,
        }
    }

    fn flush<'adapter>(&'adapter mut self) -> FlushFuture {
        FlushFuture()
    }
}

impl<'socket, 'adapter, 'buf> ReadFuture<'socket, 'adapter, 'buf> {
    fn recv(&mut self) -> Result<usize, TcpSocketAdapterError> {
        let sock = self
            .adapter
            .iface
            .get_socket::<TcpSocket>(self.adapter.handle);
        sock.recv_slice(self.buf).map_err(Into::into)
    }
}

impl<'socket, 'adapter, 'buf> Future for ReadFuture<'socket, 'adapter, 'buf> {
    type Output = Result<usize, TcpSocketAdapterError>;
    fn poll(
        mut self: core::pin::Pin<&mut Self>,
        ctx: &mut core::task::Context<'_>,
    ) -> Poll<<Self as Future>::Output> {
        if let Err(e) = self.adapter.poll() {
            return Poll::Ready(Err(e));
        }
        if !self.adapter.is_active() {
            return Poll::Ready(Err(TcpSocketAdapterError::Smoltcp(smoltcp::Error::Dropped)));
        }
        if self.adapter.can_recv() {
            Poll::Ready((*self).recv())
        } else {
            ctx.waker().wake_by_ref();
            Poll::Pending
        }
    }
}

impl<'socket, 'adapter, 'buf> WriteFuture<'socket, 'adapter, 'buf> {
    fn send(&mut self) -> Result<usize, TcpSocketAdapterError> {
        let sock = self
            .adapter
            .iface
            .get_socket::<TcpSocket>(self.adapter.handle);
        sock.send_slice(self.buf).map_err(Into::into)
    }
}

impl<'socket, 'adapter, 'buf> Future for WriteFuture<'socket, 'adapter, 'buf> {
    type Output = Result<usize, TcpSocketAdapterError>;
    fn poll(
        mut self: core::pin::Pin<&mut Self>,
        ctx: &mut core::task::Context<'_>,
    ) -> Poll<<Self as Future>::Output> {
        if let Err(e) = self.adapter.poll() {
            return Poll::Ready(Err(e));
        }
        if !self.adapter.is_active() {
            return Poll::Ready(Err(TcpSocketAdapterError::Smoltcp(smoltcp::Error::Dropped)));
        }
        if self.adapter.can_send() {
            Poll::Ready((*self).send())
        } else {
            ctx.waker().wake_by_ref();
            Poll::Pending
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

impl<'a, 'b> ConnectFuture<'a, 'b> {
    fn connect(&mut self) -> Result<(), TcpSocketAdapterError> {
        let (sock, cx) = self
            .adapter
            .iface
            .get_socket_and_context::<TcpSocket>(self.adapter.handle);
        sock.set_timeout(Some(Duration::from_secs(20)));
        sock.set_keep_alive(Some(Duration::from_secs(5)));
        sock.connect(cx, self.remote_endpoint, self.local_endpoint)
            .map_err(Into::into)
    }
}

impl<'a, 'b> Future for ConnectFuture<'a, 'b> {
    type Output = Result<(), TcpSocketAdapterError>;
    fn poll(
        mut self: core::pin::Pin<&mut Self>,
        ctx: &mut core::task::Context<'_>,
    ) -> Poll<<Self as Future>::Output> {
        if !self.connecting {
            self.connecting = true;
            match self.connect() {
                Ok(()) => {
                    ctx.waker().wake_by_ref();
                    Poll::Pending
                }
                Err(e) => Poll::Ready(Err(e)),
            }
        } else {
            if let Err(e) = self.adapter.poll() {
                return Poll::Ready(Err(e));
            }
            if self.adapter.is_active() {
                Poll::Ready(Ok(()))
            } else {
                ctx.waker().wake_by_ref();
                Poll::Pending
            }
        }
    }
}

impl<'a, 'b> Future for CloseFuture<'a, 'b> {
    type Output = Result<(), TcpSocketAdapterError>;
    fn poll(
        mut self: core::pin::Pin<&mut Self>,
        ctx: &mut core::task::Context<'_>,
    ) -> Poll<<Self as Future>::Output> {
        if !self.adapter.is_active() {
            Poll::Ready(Ok(()))
        } else {
            if let Err(e) = self.adapter.poll() {
                return Poll::Ready(Err(e));
            }
            ctx.waker().wake_by_ref();
            Poll::Pending
        }
    }
}

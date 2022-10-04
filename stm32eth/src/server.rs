use httparse::{Header, Request, Status};
use smoltcp::socket::TcpSocket;

const MAX_HEADERS: usize = 16;
const REQ_BUF_LEN: usize = 256;

pub struct Server {
    // buf: &'a mut [u8],
    // headers: Option<&'a mut [Header<'a>]>,
    // req: Option<Request<'a, 'a>>
}

pub struct Foo<'a, 'b> {
    buf: [u8; REQ_BUF_LEN],
    headers: [Header<'a>; MAX_HEADERS],
    req: Request<'a, 'b>
}

impl<'a, 'b> Foo<'a, 'b>
where 'b: 'a
{
    pub fn new() -> Self {
        Self {
            buf: [0; REQ_BUF_LEN],
            headers: [httparse::EMPTY_HEADER; MAX_HEADERS],
            req: Request::new(&headers),
        }
    }
}

impl Server {
    pub fn new() -> Self {
        Self {
            // buf,
            // headers: Some(headers),
            // req: None
        }
    }

    pub fn process(&mut self, socket: &mut TcpSocket) {
        if !socket.is_open() {
            socket.listen(80).ok();
            // if let Some(req) = self.req.take() {
            //     self.headers.replace(req.headers);
            // }
            // self.req.replace(Request::new(self.headers.take().unwrap()));
        }

        if socket.may_recv() {
            defmt::debug!("may_recv");
            let mut headers = [httparse::EMPTY_HEADER; MAX_HEADERS];
            let mut req = Request::new(&mut headers);
            let r = socket.recv(|buffer| {
                let recvd_len = buffer.len();
                let r = req.parse(buffer);
                (recvd_len, r)
            });
            let resp = match r {
                Ok(Ok(Status::Complete(_))) => {
                    // ok
                    defmt::info!("requested method: {}", req.method);
                    defmt::info!("requested path: {}", req.path);
                    if req.method == Some("GET") && req.path.is_some() {
                        get_content(req.path.unwrap())
                    } else {
                        // 405
                        "HTTP/1.1 405 Method Not Allowed\r\nContent-Type: text/plain\r\nConnection: close\r\n\r\nMethod Not Allowed"
                    }
                }
                Ok(Ok(Status::Partial)) => {
                    // 500
                    defmt::error!("partial request");
                    "HTTP/1.1 500 Internal Server Error\r\nContent-Type: text/plain\r\nConnection: close\r\n\r\nPartial request"
                }
                Ok(Err(e)) => {
                    // 400
                    defmt::error!("error in request");
                    "HTTP/1.1 400 Bad Request\r\nContent-Type: text/plain\r\nConnection: close\r\n\r\nBad Request"
                }
                Err(e) => {
                    // 500
                    defmt::error!("error in socket {}", e);
                    "HTTP/1.1 500 Internal Server Error\r\nContent-Type: text/plain\r\nConnection: close\r\n\r\nPartial request"
                }
            };
            if socket.can_send() {
                socket.send_slice(resp.as_bytes()).unwrap();
                socket.close();
            }
        } else if socket.may_send() {
            defmt::debug!("closing socket");
            socket.close();
        }
    }
}

fn get_content(path: &str) -> &'static str {
    match path {
            "/" =>
                concat!("HTTP/1.1 200 Ok\r\nContent-Type: text/html\r\nConnection: close\r\n\r\n",
                    include_str!("index.html")),
            _ =>
                "HTTP/1.1 404 Not Found\r\nContent-Type: text/plain\r\nConnection: close\r\n\r\nnot found"
        }
}

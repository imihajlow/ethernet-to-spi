use crate::adapter::{TcpSocketAdapter, TcpSocketAdapterError};
use crate::bot_token;
use crate::tg::{self, GetUpdatesResponse, SendMessageResponse};
use core::fmt::Write;
use core::str::FromStr;
use embedded_tls::*;
use heapless::String;
use heapless::Vec;
use httparse;
use rand::{CryptoRng, RngCore};
use smoltcp::wire::IpAddress;

#[derive(defmt::Format)]
pub enum TgBotError {
    Socket(TcpSocketAdapterError),
    Tls(embedded_tls::TlsError),
    HttpParseError,
    ResponseOverflow,
    HttpCodeNotOk,
    DeserializeError,
    TgNotOk,
}

pub async fn get_updates<'a, Rng: CryptoRng + RngCore>(
    offset: Option<i64>,
    timeout: Option<u16>,
    rx_buf: &'a mut [u8],
    adapter: &mut TcpSocketAdapter<'_>,
    rng: &mut Rng,
) -> Result<tg::GetUpdatesResponse<'a, 1>, TgBotError> {
    let params = tg::GetUpdatesParams {
        allowed_updates: Some(&["message"]),
        limit: Some(1),
        offset,
        timeout,
    };

    let rsp: GetUpdatesResponse<1> = api_post::<_, _, _, 128>("getUpdates", params, rx_buf, adapter, rng).await?;

    if !rsp.ok {
        return Err(TgBotError::TgNotOk);
    }

    Ok(rsp)
}

pub async fn send_message<'a, Rng: CryptoRng + RngCore>(
    chat_id: u64,
    text: &str,
    rx_buf: &'a mut [u8],
    adapter: &mut TcpSocketAdapter<'_>,
    rng: &mut Rng,
) -> Result<tg::SendMessageResponse<'a>, TgBotError> {
    let params = tg::SendMessageParams {
        chat_id,
        text
    };

    let rsp: SendMessageResponse = api_post::<_, _, _, 128>("sendMessage", params, rx_buf, adapter, rng).await?;

    if !rsp.ok {
        return Err(TgBotError::TgNotOk);
    }

    Ok(rsp)
}

async fn api_post<
    'a,
    Rng: CryptoRng + RngCore,
    Req: serde::Serialize,
    Rsp: serde::Deserialize<'a>,
    const PARAMS_LEN: usize,
>(
    method: &str,
    req: Req,
    rx_buf: &'a mut [u8],
    adapter: &mut TcpSocketAdapter<'_>,
    rng: &mut Rng,
) -> Result<Rsp, TgBotError> {
    defmt::info!("Connecting...");
    let server_ip = IpAddress::from_str("149.154.167.220").unwrap(); // TODO DNS
    let local_port: u16 = 50000 + (rng.next_u32() % 15535) as u16;
    adapter.connect((server_ip, 443), local_port).await?;

    let mut record_buffer = [0 as u8; 16384];
    let config = TlsConfig::new()
        .with_server_name("api.telegram.org")
        .verify_cert(false);
    let mut conn: TlsConnection<_, Aes128GcmSha256> =
        TlsConnection::new(adapter, &mut record_buffer);
    conn.open::<Rng, NoClock, 4096>(TlsContext::new(&config, rng))
        .await?;

    let params_json: Vec<_, PARAMS_LEN> = serde_json_core::to_vec(&req).unwrap();

    let task_result = api_post_tls(method, &params_json, rx_buf, &mut conn).await;

    let (socket, tls_close_result) = match conn.close().await {
        Ok(s) => (s, Ok(())),
        Err((s, e)) => (s, Err(e)),
    };

    let close_result: Result<_, TgBotError> = socket.close().await.map_err(Into::into);

    let rsp_data = task_result?;
    defmt::info!("rsp_data: {}", rsp_data);
    tls_close_result?;
    close_result?;

    let (rsp, _) = match serde_json_core::from_slice::<_>(rsp_data) {
        Ok(r) => r,
        Err(_) => {
            return Err(TgBotError::DeserializeError);
        }
    };
    Ok(rsp)
}

async fn api_post_tls<'a>(
    method: &str,
    data: &[u8],
    rx_buf: &'a mut [u8],
    conn: &mut TlsConnection<'_, &mut TcpSocketAdapter<'_>, Aes128GcmSha256>,
) -> Result<&'a [u8], TgBotError> {
    let mut request_str: String<256> = String::new();
    write!(&mut request_str, "POST /bot{}/{} HTTP/1.1\r\nHost: api.telegram.org\r\nContent-Length: {}\r\nContent-Type: application/json\r\n\r\n",
        bot_token::BOT_TOKEN, method, data.len()).ok();
    defmt::info!("{}", request_str);
    conn.write(request_str.as_bytes()).await?;
    conn.write(&data).await?;
    defmt::info!("{}", data);
    defmt::info!("HTTP POST Written OK");

    let rsp_len = conn.read(rx_buf).await?;
    defmt::info!("Read {} bytes", rsp_len);

    defmt::info!("Read: {}", rx_buf[..rsp_len]);

    let mut headers = [httparse::EMPTY_HEADER; 24];
    let mut rsp = httparse::Response::new(&mut headers);
    let data_offset = match rsp.parse(&rx_buf[..rsp_len]) {
        Ok(httparse::Status::Complete(len)) => len,
        Ok(httparse::Status::Partial) => return Err(TgBotError::ResponseOverflow),
        Err(e) => {
            let mut es = String::<128>::new();
            write!(es, "{:?}", e).ok();
            defmt::error!("Parse error: {}", es);
            return Err(TgBotError::HttpParseError);
        }
    };
    if rsp.code != Some(200) {
        return Err(TgBotError::HttpCodeNotOk);
    }
    Ok(&rx_buf[data_offset..rsp_len])
}

impl From<TlsError> for TgBotError {
    fn from(e: TlsError) -> Self {
        Self::Tls(e)
    }
}

impl From<TcpSocketAdapterError> for TgBotError {
    fn from(e: TcpSocketAdapterError) -> Self {
        Self::Socket(e)
    }
}

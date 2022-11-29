use crate::adapter::{TcpSocketAdapter, TcpSocketAdapterError};
use crate::bot_token;
use crate::tg;
use core::fmt::Write;
use core::str::FromStr;
use embedded_tls::*;
use heapless::String;
use heapless::Vec;
use httparse;
use rand::{CryptoRng, RngCore};
use smoltcp::wire::IpAddress;

#[derive(defmt::Format)]
enum BusinessError {
    Smoltcp(smoltcp::Error),
    Tls(embedded_tls::TlsError),
    HttpParseError,
    ResponseOverflow,
    HttpCodeNotOk,
    DeserializeError,
}

pub async fn bot_task<Rng: CryptoRng + RngCore>(
    adapter: &mut TcpSocketAdapter<'_>,
    rng: &mut Rng,
) -> () {
    // loop {
    let result = bot_step(adapter, rng).await;
    if let Err(e) = result {
        defmt::error!("Step error: {}", e);
    }
    // }
}

async fn bot_step<Rng: CryptoRng + RngCore>(
    adapter: &mut TcpSocketAdapter<'_>,
    rng: &mut Rng,
) -> Result<(), BusinessError> {
    defmt::info!("Connecting...");
    let server_ip = IpAddress::from_str("149.154.167.220").unwrap();
    let local_port: u16 = 50000 + (rng.next_u32() % 15535) as u16;
    adapter.connect((server_ip, 443), local_port).await?;

    let task_result = task(adapter, rng).await;
    let close_result: Result<_, BusinessError> = adapter.close().await.map_err(Into::into);

    task_result.and(close_result)
}

async fn task<Rng: CryptoRng + RngCore>(
    adapter: &mut TcpSocketAdapter<'_>,
    rng: &mut Rng,
) -> Result<(), BusinessError> {
    let mut record_buffer = [0 as u8; 16384];
    let mut session_id = [0 as u8; 32];
    rng.fill_bytes(&mut session_id);
    let config = TlsConfig::new()
        .with_server_name("api.telegram.org")
        .verify_cert(false);
    let mut tls: TlsConnection<_, Aes128GcmSha256> =
        TlsConnection::new(adapter, &mut record_buffer);
    tls.open::<Rng, NoClock, 4096>(TlsContext::new(&config, rng))
        .await?;
    let mut rx_buf = [0; 2048];
    let rsp = get_updates(&mut tls, &mut rx_buf).await?;
    defmt::info!("ok = {}", rsp.ok);
    defmt::info!("n entries = {}", rsp.result.len());
    for r in rsp.result.iter() {
        defmt::info!("{}", r.message.as_ref().unwrap().text.as_ref().unwrap());
    }
    Ok(())
}

async fn get_updates<'a>(
    conn: &mut TlsConnection<'_, &mut TcpSocketAdapter<'_>, Aes128GcmSha256>,
    rx_buf: &'a mut [u8],
) -> Result<tg::Response<'a, 1>, BusinessError> {
    let params = tg::GetUpdatesParams {
        allowed_updates: Some(&["message"]),
        limit: Some(1),
        offset: None,
        timeout: None,
    };
    let params_json: Vec<_, 128> = serde_json_core::to_vec(&params).unwrap();

    conn.write(b"POST /bot").await?;
    conn.write(bot_token::BOT_TOKEN).await?;
    conn.write(b"/getUpdates HTTP/1.1\r\nHost: api.telegram.org\r\nContent-Length: ")
        .await?;
    let len_string: String<4> = (params_json.len() as u16).into();
    conn.write(len_string.as_bytes()).await?;
    conn.write(b"\r\nContent-Type: application/json\r\n\r\n")
        .await?;
    conn.write(&params_json).await?;
    defmt::info!("HTTP GET Written OK");

    let rsp_len = conn.read(rx_buf).await?;
    defmt::info!("Read {} bytes", rsp_len);

    defmt::info!("Read: {}", rx_buf[..rsp_len]);

    let mut headers = [httparse::EMPTY_HEADER; 24];
    let mut rsp = httparse::Response::new(&mut headers);
    let data_offset = match rsp.parse(&rx_buf[..rsp_len]) {
        Ok(httparse::Status::Complete(len)) => len,
        Ok(httparse::Status::Partial) => return Err(BusinessError::ResponseOverflow),
        Err(e) => {
            let mut es = String::<128>::new();
            write!(es, "{:?}", e);
            defmt::error!("Parse error: {}", es);
            return Err(BusinessError::HttpParseError);
        }
    };
    if rsp.code != Some(200) {
        return Err(BusinessError::HttpCodeNotOk);
    }
    let rsp_data = &rx_buf[data_offset..rsp_len];

    let (rsp, _) = match serde_json_core::from_slice::<tg::Response<1>>(rsp_data) {
        Ok(r) => r,
        Err(_) => {
            return Err(BusinessError::DeserializeError);
        }
    };

    Ok(rsp)
}

impl From<TlsError> for BusinessError {
    fn from(e: TlsError) -> Self {
        Self::Tls(e)
    }
}

impl From<TcpSocketAdapterError> for BusinessError {
    fn from(e: TcpSocketAdapterError) -> Self {
        Self::Smoltcp(e.0)
    }
}

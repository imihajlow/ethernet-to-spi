use embedded_tls::*;
use crate::adapter::TcpSocketAdapter;
use rand::{rngs::{StdRng}, SeedableRng};

pub async fn test_tls(adapter: &mut TcpSocketAdapter<'_>) -> Result<(), TlsError> {
    let mut record_buffer = [0 as u8; 16384];
    let config = TlsConfig::new()
        .with_server_name("imihajlov.tk")
        .verify_cert(false);
    let mut tls: TlsConnection<_, Aes128GcmSha256> =
        TlsConnection::new(adapter, &mut record_buffer);
    let mut rng = StdRng::seed_from_u64(12345);
    tls.open::<StdRng, NoClock, 4096>(TlsContext::new(&config, &mut rng)).await?;
    defmt::info!("TLS open OK");
    tls.write(b"GET / HTTP/1.1\r\nHost: imihajlov.tk\r\n\r\n").await?;
    defmt::info!("HTTP GET Written OK");
    let mut rx_buf = [0; 4096];
    let size = tls.read(&mut rx_buf).await?;
    defmt::info!("Read {} bytes", size);
    defmt::info!("{}", rx_buf[0..size]);

    Ok(())
}

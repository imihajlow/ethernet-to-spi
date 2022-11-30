use heapless::String;
use crate::{adapter::TcpSocketAdapter};

use rand::{CryptoRng, RngCore};

use crate::tg_bot::{get_updates, send_message, TgBotError};

pub async fn bot_task<Rng: CryptoRng + RngCore>(
    adapter: &mut TcpSocketAdapter<'_>,
    rng: &mut Rng,
) -> ! {
    // adapter.set_timeout(Some(30000));
    let mut bot = Bot {
        adapter,
        rng,
        last_update: None,
    };
    loop {
        let result = bot.step().await;
        if let Err(e) = result {
            defmt::error!("Step error: {}", e);
        }
    }
}

struct Bot<'iface, 'a, Rng: CryptoRng + RngCore> {
    adapter: &'a mut TcpSocketAdapter<'iface>,
    rng: &'a mut Rng,
    last_update: Option<u64>,
}

impl<Rng: CryptoRng + RngCore> Bot<'_, '_, Rng> {
    async fn step(&mut self) -> Result<(), TgBotError> {
        defmt::info!("now = {}", (self.adapter.get_ticks)());
        let mut rx_buf = [0; 2048];

        let offset = self.last_update.map(|x| x as i64 + 1);

        let owned_msg = {
            let rsp = get_updates(offset, Some(20), &mut rx_buf, self.adapter, self.rng).await?;

            if rsp.result.len() > 0 {
                let update = &rsp.result[0];
                let message = &update.message;
                self.last_update = Some(update.update_id);
                if let Some(message) = &update.message {
                    let mut s = String::<128>::new();
                    if let Some(text) = message.text {
                        if s.push_str(text).is_err() {
                            s.push_str("too long :(").ok();
                        }
                    }
                    Some((message.chat.id, s))
                } else {
                    None
                }
            } else {
                None
            }
        };
        if let Some((chat_id, msg)) = owned_msg {
            send_message(chat_id, &msg, &mut rx_buf, self.adapter, self.rng).await?;
        }
        Ok(())
    }
}

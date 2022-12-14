use crate::adapter::TcpSocketAdapter;
use crate::tg_bot::{get_updates, send_message, TgBotError};
use crate::unescape::unescape;
use crate::{event, unescape};
use core::cell::RefCell;
use core::ops::DerefMut;
use core::pin::Pin;
use cortex_m::singleton;
use futures::{future::select, future::Either, pin_mut};
use heapless::{FnvIndexSet, String};
use rand::{CryptoRng, RngCore, SeedableRng, rngs::StdRng};

pub async fn bot_task(
    seed: u64,
    adapter1: TcpSocketAdapter<'_>,
    mut adapter2: TcpSocketAdapter<'_>,
    bt_press_consumer: &mut crate::event::BtnPressConsumer,
) -> ! {
    let mut rng1 = StdRng::seed_from_u64(seed);
    let mut rng2 = StdRng::seed_from_u64(rng1.next_u64());
    let send_msg_rx_buf = singleton!(: [u8; 2048] = [0; 2048]).unwrap();
    let bot = RefCell::new(Bot {
        adapter_poll: adapter1,
        rng: &mut rng1,
        last_update: None,
    });
    let mut get_msg_future_opt = None;
    let mut button_listeners = FnvIndexSet::<_, 16>::new();
    loop {
        let btn_press_future = event::wait_btn_press(bt_press_consumer);
        pin_mut!(btn_press_future);

        if get_msg_future_opt.is_none() {
            get_msg_future_opt = Some(Bot::get_message::<256>(&bot));
        }

        let get_msg_future_pin = unsafe {
            let r = get_msg_future_opt.as_mut().unwrap();
            Pin::new_unchecked(r)
        };

        match select(get_msg_future_pin, btn_press_future).await {
            Either::Left((msg_r, _btn_press_future)) => {
                match msg_r {
                    Ok(Some((chat_id, text))) => {
                        button_listeners.insert(chat_id).ok();
                        let r = send_message(chat_id, &text, send_msg_rx_buf, &mut adapter2, &mut rng2)
                            .await;
                        if let Err(e) = r {
                            defmt::error!("send_message error: {}", e);
                        }
                    }
                    Ok(None) => (),
                    Err(e) => {
                        defmt::error!("get_message error: {}", e);
                    }
                };
                get_msg_future_opt = None;
            }
            Either::Right((_ev, _r_get_msg_future)) => {
                defmt::warn!("Button is pressed");
                for chat_id in button_listeners.iter() {
                    let r = send_message(
                        *chat_id,
                        "Button is pressed!",
                        send_msg_rx_buf,
                        &mut adapter2,
                        &mut rng2,
                    )
                    .await;
                    if let Err(e) = r {
                        defmt::error!("send_message error: {}", e);
                    }
                }
            }
        };
    }
}

struct Bot<'iface, 'a, Rng: CryptoRng + RngCore> {
    adapter_poll: TcpSocketAdapter<'iface>,
    rng: &'a mut Rng,
    last_update: Option<u64>,
}

impl<Rng: CryptoRng + RngCore> Bot<'_, '_, Rng> {
    async fn get_message<const LEN: usize>(
        self_: &RefCell<Self>,
    ) -> Result<Option<(u64, String<LEN>)>, TgBotError> {
        let mut self_ref = self_.borrow_mut();
        let this = self_ref.deref_mut();
        defmt::info!("now = {}", (this.adapter_poll.get_ticks)());
        let mut rx_buf = [0; 2048];

        let offset = this.last_update.map(|x| x as i64 + 1);

        let owned_msg = {
            let rsp = {
                get_updates(
                    offset,
                    Some(20),
                    &mut rx_buf,
                    &mut this.adapter_poll,
                    &mut this.rng,
                )
                .await?
            };

            if rsp.result.len() > 0 {
                let update = &rsp.result[0];
                this.last_update = Some(update.update_id);
                if let Some(message) = &update.message {
                    if let Some(text) = message.text {
                        let ue = unescape(text);
                        if let Err(e) = &ue {
                            defmt::error!("unescape error: {:?}", e);
                        }
                        match ue {
                            Ok(s)
                            | Err(unescape::UnescapeError::Overflow(s))
                            | Err(unescape::UnescapeError::UnexpectedEnd(s)) => {
                                Some((message.chat.id, s))
                            }
                            Err(_) => {
                                let mut s = String::new();
                                s.push_str("error").unwrap();
                                Some((message.chat.id, s))
                            }
                        }
                    } else {
                        Some((message.chat.id, String::new()))
                    }
                } else {
                    None
                }
            } else {
                None
            }
        };
        Ok(owned_msg)
    }
}

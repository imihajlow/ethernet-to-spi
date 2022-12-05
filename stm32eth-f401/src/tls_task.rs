use crate::adapter::TcpSocketAdapter;
use crate::event;
use crate::tg_bot::{get_updates, send_message, TgBotError};
use core::cell::RefCell;
use core::ops::DerefMut;
use core::pin::Pin;
use cortex_m::singleton;
use futures::{future::select, future::Either, pin_mut};
use heapless::{FnvIndexSet, String};
use rand::{CryptoRng, RngCore};

pub async fn bot_task<Rng: CryptoRng + RngCore + Clone>(
    adapter1: TcpSocketAdapter<'_>,
    mut adapter2: TcpSocketAdapter<'_>,
    rng1: &mut Rng,
    rng2: &mut Rng,
    bt_press_consumer: &mut crate::event::BtnPressConsumer,
) -> ! {
    let send_msg_rx_buf = singleton!(: [u8; 2048] = [0; 2048]).unwrap();
    let bot = RefCell::new(Bot {
        adapter_poll: adapter1,
        rng: rng1,
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
                        let r = send_message(chat_id, &text, send_msg_rx_buf, &mut adapter2, rng2)
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
                        rng2,
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
                    let mut s = String::<LEN>::new();
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
        Ok(owned_msg)
    }
}

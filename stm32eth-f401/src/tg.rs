use heapless::Vec;
use serde::Deserialize;
use serde::Serialize;

#[derive(Deserialize)]
pub struct User<'a> {
    pub id: u64,
    pub is_bot: bool,
    pub first_name: &'a str,
    pub last_name: Option<&'a str>,
    pub username: Option<&'a str>,
}

#[derive(Deserialize)]
pub struct Chat<'a> {
    pub id: u64,
    #[serde(rename = "type")]
    pub chat_type: &'a str,
    pub first_name: Option<&'a str>,
    pub last_name: Option<&'a str>,
    pub username: Option<&'a str>,
}

#[derive(Deserialize)]
pub struct Message<'a> {
    pub message_id: u64,
    #[serde(borrow)]
    pub from: Option<User<'a>>,
    pub date: u64,
    #[serde(borrow)]
    pub chat: Chat<'a>,
    pub text: Option<&'a str>,
}

#[derive(Deserialize)]
pub struct Update<'a> {
    pub update_id: u64,
    #[serde(borrow)]
    pub message: Option<Message<'a>>,
}

#[derive(Deserialize)]
pub struct Response<'a, const N: usize> {
    pub ok: bool,
    #[serde(borrow)]
    pub result: Vec<Update<'a>, N>,
}


#[derive(Serialize)]
pub struct GetUpdatesParams<'a> {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub offset: Option<i64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub limit: Option<u64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub timeout: Option<u16>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub allowed_updates: Option<&'a [&'a str]>,
}

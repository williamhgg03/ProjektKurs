use heapless::String;
use serde::Deserialize;

#[derive(Debug, Deserialize)]
pub struct DisplayState {
    pub version: u64,
    pub title: String<32>,
    pub message: String<64>,
}

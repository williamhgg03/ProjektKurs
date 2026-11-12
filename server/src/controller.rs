use serde::{Deserialize, Serialize};

pub const MAX_TEXT_LENGTH: usize = 64;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DisplayState {
    pub version: u64,
    pub title: String,
    pub message: String,
}

impl Default for DisplayState {
    fn default() -> Self {
        Self {
            version: 1,
            title: "Robot controller".to_owned(),
            message: "Ready".to_owned(),
        }
    }
}

#[derive(Debug, Deserialize)]
pub struct ControllerInput {
    pub controller_id: String,
    pub action: String,
    pub x: u16,
    pub y: u16,
}

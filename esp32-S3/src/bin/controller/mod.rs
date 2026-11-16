// What the controller is: four arrow buttons on a round screen. The touch task
// turns a finger position into a `Direction`, the display task highlights it and
// the drive task sends it to the server.

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;

/// Screen size in pixels, both axes.
pub const SCREEN: i32 = 240;
pub const CENTER: i32 = SCREEN / 2;

/// Radius of the dead zone in the middle where no button is hit.
const DEAD_ZONE: i32 = 30;

#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum Direction {
    Up,
    Down,
    Left,
    Right,
}

impl Direction {
    pub const ALL: [Direction; 4] = [Self::Up, Self::Down, Self::Left, Self::Right];

    /// Body of `POST /api/segway/drive` while this button is held. Same values
    /// as the web page's defaults: lean 3 degrees, turn 0.1.
    pub fn drive_body(self) -> &'static str {
        match self {
            Self::Up => r#"{"lean_deg":3,"turn":0}"#,
            Self::Down => r#"{"lean_deg":-3,"turn":0}"#,
            Self::Left => r#"{"lean_deg":0,"turn":0.1}"#,
            Self::Right => r#"{"lean_deg":0,"turn":-0.1}"#,
        }
    }
}

/// Body of `POST /api/segway/drive` when no button is held.
pub const STOP_BODY: &str = r#"{"lean_deg":0,"turn":0}"#;

/// Which button a touch at (x, y) lands on. The screen is split into four
/// wedges by its diagonals, with a dead zone in the middle.
pub fn hit_test(x: u16, y: u16) -> Option<Direction> {
    let dx = i32::from(x) - CENTER;
    let dy = i32::from(y) - CENTER;

    if dx * dx + dy * dy < DEAD_ZONE * DEAD_ZONE {
        return None;
    }

    Some(if dy.abs() >= dx.abs() {
        if dy < 0 {
            Direction::Up
        } else {
            Direction::Down
        }
    } else if dx < 0 {
        Direction::Left
    } else {
        Direction::Right
    })
}

/// The button currently held, `None` when the finger is lifted. Written by the
/// touch task on every change; one signal per reader since a signal is consumed
/// by whoever waits on it.
pub static BUTTON_SIGNAL: Signal<CriticalSectionRawMutex, Option<Direction>> = Signal::new();
pub static DRIVE_SIGNAL: Signal<CriticalSectionRawMutex, Option<Direction>> = Signal::new();

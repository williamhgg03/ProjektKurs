// The controller's screen: four arrow buttons, one per wedge of the round
// display, with a dead zone in the middle. Matches `controller::hit_test`.

use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{Circle, Line, PrimitiveStyle, Triangle};

use crate::controller::{CENTER, Direction, SCREEN};

const BACKGROUND: Rgb565 = Rgb565::BLACK;
const BUTTON: Rgb565 = Rgb565::new(4, 10, 10);
const BUTTON_PRESSED: Rgb565 = Rgb565::new(0, 48, 12);
const ARROW: Rgb565 = Rgb565::WHITE;

/// Width of the gap between neighbouring buttons.
const SEPARATOR: u32 = 4;
/// Diameter of the empty middle.
const DEAD_ZONE_DIAMETER: u32 = 56;

/// Draw every button, with `pressed` highlighted.
pub fn draw_all<D>(target: &mut D, pressed: Option<Direction>) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    target.clear(BACKGROUND)?;
    for direction in Direction::ALL {
        draw_button(target, direction, pressed == Some(direction))?;
    }
    Ok(())
}

/// Draw one button: its wedge, the gaps to its neighbours and its arrow.
pub fn draw_button<D>(target: &mut D, direction: Direction, pressed: bool) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    let fill = if pressed { BUTTON_PRESSED } else { BUTTON };
    let center = Point::new(CENTER, CENTER);
    let last = SCREEN - 1;

    // The two screen corners the wedge reaches
    let (a, b) = match direction {
        Direction::Up => (Point::new(0, 0), Point::new(last, 0)),
        Direction::Down => (Point::new(0, last), Point::new(last, last)),
        Direction::Left => (Point::new(0, 0), Point::new(0, last)),
        Direction::Right => (Point::new(last, 0), Point::new(last, last)),
    };

    Triangle::new(center, a, b)
        .into_styled(PrimitiveStyle::with_fill(fill))
        .draw(target)?;

    // Gaps along the diagonals and the dead zone in the middle
    let gap = PrimitiveStyle::with_stroke(BACKGROUND, SEPARATOR);
    Line::new(center, a).into_styled(gap).draw(target)?;
    Line::new(center, b).into_styled(gap).draw(target)?;
    Circle::with_center(center, DEAD_ZONE_DIAMETER)
        .into_styled(PrimitiveStyle::with_fill(BACKGROUND))
        .draw(target)?;

    // Arrow pointing outwards, tip 85 px from the centre, base 45 px from it
    let (tip, base_l, base_r) = match direction {
        Direction::Up => (Point::new(120, 35), Point::new(95, 75), Point::new(145, 75)),
        Direction::Down => (
            Point::new(120, 204),
            Point::new(95, 164),
            Point::new(145, 164),
        ),
        Direction::Left => (Point::new(35, 120), Point::new(75, 95), Point::new(75, 145)),
        Direction::Right => (
            Point::new(204, 120),
            Point::new(164, 95),
            Point::new(164, 145),
        ),
    };
    Triangle::new(tip, base_l, base_r)
        .into_styled(PrimitiveStyle::with_fill(ARROW))
        .draw(target)
}

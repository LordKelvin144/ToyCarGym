use macroquad::prelude as mq;

use super::super::physics::{CarState, CarConfig};
use crate::math_utils::Vec2;


// Scale factor -> Bigger means simulation units look larger
const SCALE: f32 = 10.0;

// Ratio width/length of car graphic
const WIDTH_RATIO: f32 = 0.5;

// Relative length from back of car to back axle
const BACK_AXLE: f32 = 0.2;

// Relative length from back to front axle
const FRONT_AXLE: f32 = 0.8;


pub fn draw_car(state: &CarState, config: &CarConfig) {
    let (W, H) = (mq::screen_width(), mq::screen_height());

    // Car position represents the position of the center of the back axle
    // A physical coordinate of (0,0) should be at the center of the screen
    let back_axle_center = Vec2(state.position.0*SCALE, -state.position.1*SCALE) + Vec2(W/2.0, H/2.0);

    let unit_forward = Vec2(state.velocity.0, -state.velocity.1).normalized();
    let unit_left = -unit_forward.rotate90();

    let width = config.length * WIDTH_RATIO;
    let back_center = back_axle_center + unit_forward * (-BACK_AXLE*config.length*SCALE);
    let back_left_corner = back_axle_center + unit_left * (0.5*width*SCALE);
    let back_right_corner = back_axle_center + unit_left * (-0.5*width*SCALE);

    let forward_left_corner = back_left_corner + unit_forward * config.length * SCALE;
    let forward_right_corner = back_right_corner + unit_forward * config.length * SCALE;

    mq::draw_line(back_left_corner.0, back_left_corner.1, forward_left_corner.0, forward_left_corner.1, 3.0, mq::RED);
    mq::draw_line(forward_left_corner.0, forward_left_corner.1, forward_right_corner.0, forward_right_corner.1, 3.0, mq::GREEN);
    mq::draw_line(forward_right_corner.0, forward_right_corner.1, back_right_corner.0, back_right_corner.1, 3.0, mq::RED);
    mq::draw_line(back_right_corner.0, back_right_corner.1, back_left_corner.0, back_left_corner.1, 3.0, mq::GREEN);
}


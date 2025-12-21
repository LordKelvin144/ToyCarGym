use macroquad::prelude as mq;

use super::super::physics::{CarState, CarConfig, CarInput};
use crate::math_utils::Vec2;


// Scale factor -> Bigger means simulation units look larger
const SCALE: f32 = 10.0;

// Ratio width/length of car graphic
const WIDTH_RATIO: f32 = 0.5;

// Relative length from back of car to back axle
const BACK_AXLE: f32 = 0.2;

// Relative length from back to front axle
const FRONT_AXLE: f32 = 0.8;
const WHEEL_LENGTH: f32 = 0.2;


pub fn draw_car(state: &CarState, input: &CarInput, config: &CarConfig) {
    let (width, height) = (mq::screen_width(), mq::screen_height());

    // Car position represents the position of the center of the back axle
    // A physical coordinate of (0,0) should be at the center of the screen
    let back_axle_center = Vec2(state.position.0*SCALE, -state.position.1*SCALE) + Vec2(width/2.0, height/2.0);

    let unit_forward = Vec2(state.velocity.0, -state.velocity.1).normalized();
    let unit_left = -unit_forward.rotate90();

    let width = config.length * WIDTH_RATIO;

    let left_half_width = unit_left * 0.5*width*SCALE;

    // Compute relevant positions
    let back_center = back_axle_center - unit_forward * (BACK_AXLE*config.length*SCALE);

    let bl_corner = back_center + left_half_width;
    let br_corner = back_center - left_half_width;
    let fl_corner = bl_corner + unit_forward * config.length * SCALE;
    let fr_corner = br_corner + unit_forward * config.length * SCALE;

    // Compute centers of wheels
    let lb_wheel = back_axle_center + left_half_width*1.05;
    let rb_wheel = back_axle_center - left_half_width*1.05;
    let lf_wheel = lb_wheel + unit_forward * (FRONT_AXLE-BACK_AXLE)*config.length*SCALE;
    let rf_wheel = rb_wheel + unit_forward * (FRONT_AXLE-BACK_AXLE)*config.length*SCALE;

    let draw_wheel = |center: Vec2<_>, angle: f32| {
        let unit_wheel = unit_forward.rotate(-angle);
        let back_point = center - unit_wheel*0.5*WHEEL_LENGTH*config.length*SCALE;
        let front_point = center + unit_wheel*0.5*WHEEL_LENGTH*config.length*SCALE;
        mq::draw_line(back_point.0, back_point.1, front_point.0, front_point.1, 5.0, mq::BLACK);
    };

    // Draw the wheels
    draw_wheel(lb_wheel, 0.0);
    draw_wheel(rb_wheel, 0.0);
    draw_wheel(lf_wheel, input.steer_delta);
    draw_wheel(rf_wheel, input.steer_delta);

    // Draw the car
    mq::draw_triangle(bl_corner.into(),
                      fl_corner.into(),
                      fr_corner.into(), mq::BLUE);
    mq::draw_triangle(fr_corner.into(),
                      br_corner.into(),
                      bl_corner.into(), mq::BLUE);

    mq::draw_line(bl_corner.0, bl_corner.1, fl_corner.0, fl_corner.1, 3.0, mq::RED);
    mq::draw_line(fl_corner.0, fl_corner.1, fr_corner.0, fr_corner.1, 3.0, mq::GREEN);
    mq::draw_line(fr_corner.0, fr_corner.1, br_corner.0, br_corner.1, 3.0, mq::RED);
    mq::draw_line(br_corner.0, br_corner.1, bl_corner.0, bl_corner.1, 3.0, mq::GREEN);
}


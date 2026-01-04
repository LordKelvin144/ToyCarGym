use macroquad::prelude as mq;

use super::super::physics::{CarState, CarConfig, CarInput};
use super::super::map::{CellMap, SplineMap, LidarArray, DrawRoad};
use crate::math_utils::Vec2;
use crate::graphics_utils::{ScreenTransform, draw_spline};

// Ratio width/length of car graphic
const WIDTH_RATIO: f32 = 0.5;

// Relative length of wheel to display
const WHEEL_LENGTH: f32 = 0.2;


pub fn draw_car(state: &CarState, input: &CarInput, config: &CarConfig, transform: &ScreenTransform) {
    // Car position represents the position of the center of the back axle
    // A physical coordinate of (0,0) should be at the center of the screen

    let back_axle_center = state.position;
    let back_center = state.position - state.unit_forward * config.back_axle;
    let unit_left = state.unit_forward.rotate90();
    let half_lateral_displacement = unit_left*0.5*config.length*WIDTH_RATIO;
    let forward_displacement = state.unit_forward*config.length;

    // Get positions of car corners
    let bl_corner = back_center + half_lateral_displacement;
    let br_corner = back_center - half_lateral_displacement;
    let fl_corner = bl_corner + forward_displacement;
    let fr_corner = br_corner + forward_displacement;

    // Get wheel positions
    let bl_wheel = back_axle_center + half_lateral_displacement*1.1;
    let br_wheel = back_axle_center - half_lateral_displacement*1.1;
    let front_wheel_to_back_wheel = state.unit_forward*(config.front_axle-config.back_axle);
    let fl_wheel = bl_wheel + front_wheel_to_back_wheel;
    let fr_wheel = br_wheel + front_wheel_to_back_wheel;

    let draw_wheel = |center: Vec2<f32>, angle: f32| {
        let unit_wheel = state.unit_forward.rotate(angle);
        let half_wheel = unit_wheel*0.5*WHEEL_LENGTH * config.length;

        let back_point = transform.to_screen(center - half_wheel);
        let front_point = transform.to_screen(center + half_wheel);
        mq::draw_line(back_point.x, back_point.y, front_point.x, front_point.y, 5.0, mq::BLACK);
    };

    // Draw the wheels
    draw_wheel(bl_wheel, 0.0);
    draw_wheel(br_wheel, 0.0);
    draw_wheel(fl_wheel, input.steer_delta);
    draw_wheel(fr_wheel, input.steer_delta);

    // Draw the car
    let bl_corner = transform.to_screen(bl_corner);
    let br_corner = transform.to_screen(br_corner);
    let fl_corner = transform.to_screen(fl_corner);
    let fr_corner = transform.to_screen(fr_corner);
    mq::draw_triangle(bl_corner.into(),
                      fl_corner.into(),
                      fr_corner.into(), mq::BLUE);
    mq::draw_triangle(fr_corner.into(),
                      br_corner.into(),
                      bl_corner.into(), mq::BLUE);

    mq::draw_line(bl_corner.x, bl_corner.y, fl_corner.x, fl_corner.y, 3.0, mq::RED);
    mq::draw_line(fl_corner.x, fl_corner.y, fr_corner.x, fr_corner.y, 3.0, mq::GREEN);
    mq::draw_line(fr_corner.x, fr_corner.y, br_corner.x, br_corner.y, 3.0, mq::RED);
    mq::draw_line(br_corner.x, br_corner.y, bl_corner.x, bl_corner.y, 3.0, mq::GREEN);
}


impl DrawRoad for CellMap {
    fn draw_road(&self, transform: &ScreenTransform) {
        for i in 0 .. &self.cells.len()-1 {
            let cell1 = &self.cells[i];
            let cell2 = &self.cells[i+1];

            let left = cell1.0.min(cell2.0) as f32 - 0.45;
            let right = cell1.0.max(cell2.0) as f32 + 0.45;
            let bottom = cell1.1.min(cell2.1) as f32 - 0.45;
            let top = cell1.1.max(cell2.1) as f32 + 0.45;

            let top_left = transform.to_screen(Vec2(left, top)*self.cell_size);
            let top_right = transform.to_screen(Vec2(right, top)*self.cell_size);
            let bottom_left = transform.to_screen(Vec2(left, bottom)*self.cell_size);
            let bottom_right = transform.to_screen(Vec2(right, bottom)*self.cell_size);

            mq::draw_triangle(top_left, top_right, bottom_right, mq::GRAY);
            mq::draw_triangle(bottom_right, bottom_left, top_left, mq::GRAY);
        }
    }
}


impl DrawRoad for SplineMap {
    fn draw_road(&self, transform: &ScreenTransform) {
        let segments = 64;
        let color = mq::Color { r: 0.3, g: 0.3, b: 0.3, a: 1.0 };
        draw_spline(&self.spline, transform, self.width, segments, color);
        draw_spline(&self.spline, transform, 0.1, segments, mq::WHITE);
    }
}


pub fn draw_lidar(state: &CarState, lidar: &LidarArray, readings: &[f32], transform: &ScreenTransform) {
    // Car position represents the position of the center of the back axle
    // A physical coordinate of (0,0) should be at the center of the screen
    let lidar_pos = state.position;
    let lidar_pos_screen = transform.to_screen(state.position);

    let points = lidar.get_angles().into_iter().zip(readings)
        .map(|(&angle, &reading)| {
            let direction = state.unit_forward.rotate(angle);
            transform.to_screen(lidar_pos + direction*reading)
        });

    for point in points {
        mq::draw_line(lidar_pos_screen.x, lidar_pos_screen.y, point.x, point.y, 1.0, mq::Color{r: 0.6, g: 0.0, b: 0.0, a: 0.5});
    }
}


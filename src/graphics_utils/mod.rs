use macroquad::prelude as mq;

use crate::math_utils::spline;


pub fn draw_bezier(curve: &spline::CubicBezier, segments: usize, width: f32, color: mq::Color) {
    let dt = 1.0 / (segments as f32);
    let mut t = 0.0;

    for _ in 0 .. segments {
        let start = curve.get(0.0_f32.max(t-0.25*dt));
        let end = curve.get(1.0_f32.min(t+1.25*dt));
        mq::draw_line(start.0, start.1, end.0, end.1, width, color);

        t += dt;
    }
}

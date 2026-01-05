use macroquad::prelude as mq;

use math_utils::{Vec2, spline};


pub struct ScreenTransform {
    px_per_m: f32,
    center_m: Vec2<f32>,
}


impl ScreenTransform {
    pub fn new(px_per_m: f32) -> Self {
        Self { px_per_m, center_m: Vec2(0.0, 0.0) }
    }

    pub fn to_screen(&self, world: Vec2<f32>) -> mq::Vec2 {
        let center_px = mq::Vec2 { x: mq::screen_width()*0.5, y: mq::screen_height()*0.5 };
        let world_center_dev = world - self.center_m;

        mq::Vec2 { x: world_center_dev.0 * self.px_per_m, y: -world_center_dev.1*self.px_per_m } + center_px
    }

    pub fn set_center(&mut self, center: Vec2<f32>) {
        self.center_m = center;
    }
}


pub fn draw_bezier(curve: &spline::CubicBezier, transform: &ScreenTransform, world_width: f32, segments: usize, color: mq::Color) {
    let dt = 1.0 / (segments as f32);
    let mut t = dt;

    // Define a closure which takes in a scalar parameter and computes the screen coordinates of
    // (left_edge, center, right_edge) as given by the normal at t
    //
    //  l---c---r
    //  | / | \ |
    //  ll--cl--rl
    //
    let get_segment = |t: f32| -> (mq::Vec2, mq::Vec2, mq::Vec2) {
        let point = curve.get(t);
        let normal = curve.tangent(t).rotate90();
        let c = transform.to_screen(point);
        let l = transform.to_screen(point + normal*world_width*0.5);
        // r = c - (l-c) 
        let r = c*2.0 - l;
        (l, c, r)
    };

    let (mut last_l, mut last_c, mut last_r) = get_segment(0.0);

    for _ in 1 ..= segments {
        let (l, c, r) = get_segment(t);
        // Draw triangles according to 
        //
        //  l---c---r
        //  | / | \ |
        //  ll--cl--rl

        // Bottom left triangle
        mq::draw_triangle(last_l, last_c, c, color);

        // Bottom right triangle
        mq::draw_triangle(last_r, last_c, c, color);

        // Top left triangle
        mq::draw_triangle(l, c, last_l, color);

        // Top right triangle
        mq::draw_triangle(r, c, last_r, color);

        last_l = l;
        last_c = c;
        last_r = r;
        t += dt;
    }
}


pub fn draw_spline(spline: &spline::SmoothBezierSpline, transform: &ScreenTransform, world_width: f32, segments: usize, color: mq::Color) {
    let sub_segments = segments / spline.segments.len() + 1;
    for segment in &spline.segments {
        draw_bezier(segment, transform, world_width, sub_segments, color);
    };
}

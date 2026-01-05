use crate::math_utils::{
    Vec2, 
    spline::{SmoothBezierSpline, ClosestPointOutput, BezierControl},
    root::find_root,
};

use crate::car::physics::{CarState, CarConfig};
use super::traits::Road;

pub struct SplineMap {
    pub spline: SmoothBezierSpline,
    pub width: f32,
    max_d2: f32,
}


impl SplineMap {
    fn new(spline: SmoothBezierSpline, width: f32) -> Self {
        let max_d2 = 0.25*width*width;
        SplineMap { spline, width, max_d2 }
    }

    fn point_inside(&self, point: Vec2<f32>) -> bool {
        let ClosestPointOutput { distance_sq, ..} = self.spline.closest_point(point);
        distance_sq < self.max_d2
    }
}


impl Road for SplineMap {
    fn is_crashed(&self, state: &CarState, config: &CarConfig) -> bool {
        // Check if both the back and front points are inside the road;
        let back_point = state.position - state.unit_forward*config.back_axle;
        let front_point = back_point + state.unit_forward*config.length;
        !self.point_inside(back_point) || !self.point_inside(front_point)
    }

    /// Takes in a point and (non-normalized) direction defining a ray,
    /// and finds the first intersection with the edge of the track.
    fn ray_collision(&self, point: Vec2::<f32>, direction: Vec2::<f32>) -> Vec2::<f32> {  
        let step_length = self.width * 0.1;
        let step = direction.normalized() * step_length;
        let mut p = point;

        // Early return if we have already crashed
        if !self.point_inside(point) {
            return point;
        }

        // Find the a point 'inside_point' such that 'inside_point' is inside the road
        // and inside_point + step is outside
        let inside_point = loop {
            let next_p = p + step;
            if !self.point_inside(next_p) {
                break p
            }
            p = next_p;
        };

        // Do bisection to find the exact position of the edge
        //
        // Define a function f(t) such that f(t) is zero at t such that inside_point * t*step is on
        // the edge
        let edge_deviation = |t| self.spline.closest_point(inside_point + step*t).distance_sq - self.max_d2;
        let t = find_root(edge_deviation, 0.0, 1.0, 1e-2).expect("the prior code to ensure a root exists");
        inside_point + step*t
    }
}


pub fn make_oval() -> SplineMap {
    let spline = SmoothBezierSpline::new(
        vec![BezierControl{ point: Vec2(0.0, 0.0), velocity: Vec2(6.0, 0.0)},
             BezierControl{ point: Vec2(10.0, 10.0), velocity: Vec2(0.0, 6.0)},
             BezierControl{ point: Vec2(0.0, 20.0), velocity: Vec2(-6.0, 0.0)},
             BezierControl{ point: Vec2(-20.0, 20.0), velocity: Vec2(-6.0, 0.0)},
             BezierControl{ point: Vec2(-30.0, 10.0), velocity: Vec2(0.0, -6.0)},
             BezierControl{ point: Vec2(-20.0, 0.0), velocity: Vec2(6.0, 0.0)},
             BezierControl{ point: Vec2(0.0, 0.0), velocity: Vec2(6.0, 0.0)}]
    );
    let width = 8.0;
    SplineMap::new(spline, width)
}

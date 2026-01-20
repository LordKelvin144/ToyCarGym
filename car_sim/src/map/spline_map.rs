use math_utils::{
    Vec2, 
    spline::{SmoothBezierSpline, ClosestPointOutput, BezierControl},
    root::find_root,
};

use crate::physics::{CarState, CarConfig};
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

pub fn make_simple_racetrack() -> SplineMap {
    let spline = SmoothBezierSpline::new(
        vec![
            BezierControl { point: Vec2(0.0, 0.0), velocity: Vec2(50.0, -10.0) },
            BezierControl { point: Vec2(87.5, 50.0), velocity: Vec2(0.0, 25.0) },
            BezierControl { point: Vec2(50.0, 150.0), velocity: Vec2(-40.0, 10.0) },
            BezierControl { point: Vec2(-25.0, 100.0), velocity: Vec2(0.0, -40.0) },
            BezierControl { point: Vec2(50.0, 75.0), velocity: Vec2(25.0, -25.0) },
            BezierControl { point: Vec2(0.0, 50.0), velocity: Vec2(-20.0, 0.0) },
            BezierControl { point: Vec2(0.0, 0.0), velocity: Vec2(50.0, -10.0) },
        ]
    );
    let width = 10.0;
    SplineMap::new(spline, width)
}

pub fn make_racetrack() -> SplineMap {
    let spline = SmoothBezierSpline::new(
        vec![
            BezierControl { point: Vec2(0.0, 0.0), velocity: Vec2(-30.0, 4.0) },
            BezierControl { point: Vec2(-168.4, 24.5), velocity: Vec2(-30.0, 4.0) },
            BezierControl { point: Vec2(-246.0, 36.6), velocity: Vec2(-6.0, 6.0) },
            BezierControl { point: Vec2(-260.0, 67.6), velocity: Vec2(-6.0, 6.0) },
            BezierControl { point: Vec2(-296.0, 77.6), velocity: Vec2(-8.0, 8.0) },
            BezierControl { point: Vec2(-342.5, 192.6), velocity: Vec2(-8.0, 30.0) },
            BezierControl { point: Vec2(-365.0, 300.6), velocity: Vec2(-8.0, 16.0) },
            BezierControl { point: Vec2(-417.5, 400.8), velocity: Vec2(-2.0, 16.0) },
            BezierControl { point: Vec2(-402.0, 437.6), velocity: Vec2(10.0, 8.0) },
            BezierControl { point: Vec2(-341.0, 445.6), velocity: Vec2(24.0, -10.0) },
            BezierControl { point: Vec2(-189.0, 367.6), velocity: Vec2(32.0, -18.0) },
            BezierControl { point: Vec2(-56.8, 277.2), velocity: Vec2(12.0, -4.0) },
            BezierControl { point: Vec2(44.0, 256.6), velocity: Vec2(8.0, -3.0) },
            BezierControl { point: Vec2(67.0, 220.6), velocity: Vec2(0.0, -20.0) },
            BezierControl { point: Vec2(33.0, 182.6), velocity: Vec2(-14.0, -4.0) },
            BezierControl { point: Vec2(-56.1, 194.9), velocity: Vec2(-19.0, 10.0) },
            BezierControl { point: Vec2(-147.5, 252.0), velocity: Vec2(-15.0, 4.0) },
            BezierControl { point: Vec2(-177.1, 249.1), velocity: Vec2(-7.0, -4.0) },
            BezierControl { point: Vec2(-194.3, 226.9), velocity: Vec2(-3.0, -10.0) },
            BezierControl { point: Vec2(-196.8, 146.9), velocity: Vec2(4.0, -9.0) },
            BezierControl { point: Vec2(-161.0, 118.6), velocity: Vec2(16.0, 0.0) },
            BezierControl { point: Vec2(-119.3, 132.1), velocity: Vec2(8.0, 4.0) },
            BezierControl { point: Vec2(-71.2, 145.1), velocity: Vec2(15.0, 0.0) },
            BezierControl { point: Vec2(-32.5, 135.2), velocity: Vec2(8.0, -3.2) },
            BezierControl { point: Vec2(156.6, 57.3), velocity: Vec2(8.0, -3.2) },
            BezierControl { point: Vec2(176.2, 33.9), velocity: Vec2(0.0, -15.0) },
            BezierControl { point: Vec2(152.8, -3.1), velocity: Vec2(-12.0, -6.0) },
            BezierControl { point: Vec2(101.7, -13.7), velocity: Vec2(-12.0, 0.0) },
            BezierControl { point: Vec2(0.0, 0.0), velocity: Vec2(-30.0, 4.0) },
        ]
    );
    let width = 10.0;
    SplineMap::new(spline, width)
}

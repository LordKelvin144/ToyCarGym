use super::vec::Vec2;
use itertools::Itertools;


// TODO: 
//   - Given a point, find the closest point (or its parameter) on the spline
//   - Given a parameter, find the arc length
//   - Given an arc length, find the parameter


pub struct CubicBezier {
    pub start: Vec2<f32>,
    pub p1: Vec2<f32>,
    pub p2: Vec2<f32>,
    pub end: Vec2<f32>,
    c1: Vec2<f32>,
    c2: Vec2<f32>,
    c3: Vec2<f32>,
}


/// Represents a single spline point and its tangent velocity specification
pub struct BezierControl {
    pub point: Vec2<f32>,
    pub velocity: Vec2<f32>,
}


pub struct SmoothBezierSpline {
    pub segments: Vec<CubicBezier>,
    pub max_u: f32,
}


impl CubicBezier {
    pub fn new(start: Vec2<f32>, 
           p1: Vec2<f32>, 
           p2: Vec2<f32>,
           end: Vec2<f32>) -> Self {
        let c1 = (p1-start)*3.0;
        let c2 = start*3.0 - p1*6.0 + p2*3.0;
        let c3 = -start + p1*3.0 - p2*3.0 + end;
        CubicBezier {
            start, p1, p2, end,
            c1, c2, c3
        }
    }

    pub fn get(&self, t: f32) -> Vec2<f32> {
        self.start + self.c1 * t + self.c2*t*t + self.c3 * t*t*t
    }

    pub fn velocity(&self, t: f32) -> Vec2<f32> {
        self.c1 + self.c2 * 2.0 * t + self.c3 * 3.0*t*t
    }

    pub fn tangent(&self, t: f32) -> Vec2<f32> {
        self.velocity(t).normalized()
    }
}


impl SmoothBezierSpline {
    pub fn new(controls: Vec<BezierControl>) -> Self {
        let segments: Vec<CubicBezier> = controls.iter()
            .tuple_windows()
            .map(|(control_start, control_end)| {
                CubicBezier::new(control_start.point, 
                                 control_start.point + control_start.velocity,
                                 control_end.point - control_end.velocity,
                                 control_end.point)
            })
            .collect();

        let max_u = segments.len() as f32;
        Self { segments, max_u }
    }

    pub fn get(&self, u: f32) -> Vec2<f32> {
        let u = u.min(self.max_u).max(0.0);
        let i = u as usize;
        let t = u.fract();

        // Edge case 
        if u == self.max_u {
            return self.segments[self.segments.len()-1].end;
        }

        let segment = &self.segments[i];
        segment.get(t)
    }

    pub fn velocity(&self, u: f32) -> Vec2<f32> {
        let u = u.min(self.max_u).max(0.0);
        let i = u as usize;
        let t = u.fract();

        // Edge case 
        if u == self.max_u {
            return self.segments[self.segments.len()-1].velocity(1.0);
        }

        let segment = &self.segments[i];
        segment.velocity(t)
    }

    pub fn tangent(&self, u: f32) -> Vec2<f32> {
        self.velocity(u).normalized()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_limits() {
        let start = Vec2(-1.0, 0.0);
        let p1 = Vec2(-1.0, 1.0);
        let p2 = Vec2(1.0, 1.0);
        let end = Vec2(1.0, 0.0);
        let spline = CubicBezier::new(start, p1, p2, end);
        assert_eq!(spline.get(0.0), start);
        assert_eq!(spline.get(1.0), end);
        assert_eq!(spline.get(0.5).0, 0.0);  // Control point symmetric about x axis, so 0.5 should
                                             // have x=0

        let spline = SmoothBezierSpline::new(vec![BezierControl{ point: Vec2(0.0, 0.0), velocity: Vec2(0.0, 1.0)},
                                                  BezierControl{ point: Vec2(1.0, 0.0), velocity: Vec2(0.0, -1.0)},
                                                  BezierControl{ point: Vec2(2.0, 0.0), velocity: Vec2(0.0, 1.0)}]);
        assert_eq!(spline.get(0.0), Vec2(0.0, 0.0));
        assert_eq!(spline.get(0.5).0, 0.5);
        assert_eq!(spline.get(1.0), Vec2(1.0, 0.0));
        assert_eq!(spline.get(2.0), Vec2(2.0, 0.0));
    } 

    #[test]
    fn test_velocity() {
        let t = 0.625;  // Arbitrary

        let start = Vec2(-1.0, 0.0);
        let p1 = Vec2(-1.0, 1.0);
        let p2 = Vec2(1.0, 1.0);
        let end = Vec2(1.0, 0.0);
        let spline = CubicBezier::new(start, p1, p2, end);

        // Check that velocity is consistent with finite difference approximation
        let epsilon = 0.00001;
        let c = spline.get(t);
        let cp = spline.get(t + epsilon);

        let v_fd = (cp - c) / epsilon;
        let v = spline.velocity(t);

        let error = (v_fd - v).norm() / v.norm();
        assert!(error < 0.01);

        assert_eq!(spline.velocity(0.0).0, 0.0);
        assert_eq!(spline.velocity(1.0).0, 0.0);
        assert_eq!(spline.velocity(0.5).1, 0.0);
    }

    #[test]
    fn test_tangent() {
        let start = Vec2(-1.0, 0.0);
        let p1 = Vec2(-1.0, 1.0);
        let p2 = Vec2(1.0, 1.0);
        let end = Vec2(1.0, 0.0);
        let spline = CubicBezier::new(start, p1, p2, end);

        assert_eq!(spline.tangent(0.0), Vec2(0.0, 1.0));
        assert_eq!(spline.tangent(0.5), Vec2(1.0, 0.0));
        assert_eq!(spline.tangent(1.0), Vec2(0.0, -1.0));

        let spline = SmoothBezierSpline::new(vec![BezierControl{ point: Vec2(0.0, 0.0), velocity: Vec2(0.0, 1.0)},
                                                  BezierControl{ point: Vec2(1.0, 0.0), velocity: Vec2(0.0, -1.0)},
                                                  BezierControl{ point: Vec2(2.0, 0.0), velocity: Vec2(0.0, 1.0)}]);
        assert_eq!(spline.tangent(0.0), Vec2(0.0, 1.0));
        assert_eq!(spline.tangent(0.5), Vec2(1.0, 0.0));
        assert_eq!(spline.tangent(1.0), Vec2(0.0, -1.0));
        assert_eq!(spline.tangent(2.0), Vec2(0.0, 1.0));
    }
}



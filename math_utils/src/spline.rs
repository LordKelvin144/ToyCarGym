use super::vec::Vec2;
use itertools::Itertools;

use std::cmp::Ordering;

use super::root::{FunctionObservation, find_min_differentiable};


pub struct CubicBezier {
    pub start: Vec2<f32>,
    pub p1: Vec2<f32>,
    pub p2: Vec2<f32>,
    pub end: Vec2<f32>,
    c1: Vec2<f32>,
    c2: Vec2<f32>,
    c3: Vec2<f32>,
    arc_length: f32,
    bounding_box: BoundingBox,
}


#[derive(Debug)]
struct BoundingBox {
    pub min_x: f32,
    pub max_x: f32,
    pub min_y: f32,
    pub max_y: f32,
    pub corners: [Vec2<f32>; 4]
}

impl BoundingBox {
    fn new(min_x: f32, max_x: f32, min_y: f32, max_y: f32) -> Self {
        let corners = [Vec2(min_x, min_y), Vec2(min_x, max_y), Vec2(max_x, min_y), Vec2(max_x, max_y)];
        Self { min_x, max_x, min_y, max_y, corners }
    }

    pub fn closest_point(&self, point: Vec2::<f32>) -> Vec2::<f32> {
        let Vec2(x, y) = point;
        match (x <= self.min_x, x > self.max_x, y <= self.min_y, y > self.max_y) {
            // Corner quadrants
            (true, _, true, _) => Vec2(self.min_x, self.min_y),
            (true, _, _, true) => Vec2(self.min_x, self.max_y),
            (_, true, _, true) => Vec2(self.max_x, self.max_y),
            (_, true, true, _) => Vec2(self.max_x, self.min_y),
            
            // Middle quadrant
            (false, false, false, false) => Vec2(x, y),

            // Edge quadrants
            (true, false, false, false) => Vec2(self.min_x, y),
            (false, true, false, false) => Vec2(self.max_x, y),
            (false, false, true, false) => Vec2(x, self.min_y),
            (false, false, false, true) => Vec2(x, self.max_y),

            (true, true, _, _) => panic!("Impossible quadrant! (x,y)=({},{}), bbox={:?}", x, y, self),
            (_, _, true, true) => panic!("Impossible quadrant! (x,y)=({},{}), bbox={:?}", x, y, self),
        }
    }

    pub fn farthest_point(&self, point: Vec2::<f32>) -> Vec2::<f32> {
        let (corner, _d2) = self.corners.iter()
            .map(|corner| {
                let delta = point - *corner;
                let d2 = delta.dot(delta);
                (*corner, d2)
            })
            .reduce(|(corner, d2): (Vec2<_>, f32), (new_corner, new_d2): (Vec2<_>, f32)| match new_d2.total_cmp(&d2) {
                Ordering::Greater => (new_corner, new_d2),
                _ => (corner, d2),
            })
            .expect("at least one corner to exist");
        corner
    }
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


#[derive(Debug, PartialEq)]
pub struct ClosestPointOutput {
    pub parameter: f32,
    pub distance_sq: f32
}


impl CubicBezier {
    pub fn new(start: Vec2<f32>, 
           p1: Vec2<f32>, 
           p2: Vec2<f32>,
           end: Vec2<f32>) -> Self {
        let c1 = (p1-start)*3.0;
        let c2 = start*3.0 - p1*6.0 + p2*3.0;
        let c3 = -start + p1*3.0 - p2*3.0 + end;
        let mut this = CubicBezier {
            start, p1, p2, end,
            c1, c2, c3,
            arc_length: 0.0,
            bounding_box: BoundingBox::new(0.0, 0.0, 0.0, 0.0),
        };
        this.arc_length = this._arc_length(0.0, 1.0, 32);
        this.bounding_box = this._bounding_box();
        this
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

    fn _arc_length(&self, t_start: f32, t_end: f32, steps: usize) -> f32 {
        // Arc length is int_{t_start}^{t_end} |velocity(t)|dt
        // Compute it numerically using trapezoid method
        let dt = (t_end - t_start) / steps as f32;
        let ts = (1 .. steps).map(|i| t_start + i as f32*dt);  // [dt, 2*dt, ..., t-dt]
                                                               //
        ts.map(|t| self.velocity(t).norm()*dt).sum::<f32>() + 0.5*dt*(self.velocity(t_start).norm() + self.velocity(t_end).norm())
    }

    // Computes the tangential arc length from t=0 to t=t
    pub fn arc_length(&self, t: f32) -> f32 {
        if t == 1.0 {
            self.arc_length
            
        } else {
            self._arc_length(0.0, t, 32)
        }
    }

    fn _bounding_box(&self) -> BoundingBox {
        let fx = |t| { self.get(t).0 };
        let fpx = |t| { self.velocity(t).0 };
        let min_x = find_min_differentiable(fx, fpx, 0.0, 1.0, 1e-4).value;
        let max_x = -find_min_differentiable(|t| -fx(t), |t| -fpx(t), 0.0, 1.0, 1e-4).value;

        let fy = |t| { self.get(t).1 };
        let fpy = |t| { self.velocity(t).1 };

        let min_y = find_min_differentiable(fy, fpy, 0.0, 1.0, 1e-4).value;
        let max_y = -find_min_differentiable(|t| -fy(t), |t| -fpy(t), 0.0, 1.0, 1e-4).value;
        BoundingBox::new(min_x, max_x, min_y, max_y)
    }

    pub fn closest_point(&self, point: Vec2<f32>) -> ClosestPointOutput {
        let f = |t| {
            let pt = self.get(t);
            (pt - point).dot(pt-point)
        };

        let fp = |t| {
            let pt = self.get(t);
            let v = self.velocity(t);
            (pt - point).dot(v) * 2.0
        };

        let FunctionObservation { x: t, value: distance_sq, ..} = find_min_differentiable(f, fp, 0.0, 1.0, 1e-2);
        ClosestPointOutput { parameter: t, distance_sq }
    }
}


impl SmoothBezierSpline {
    pub fn new(controls: Vec<BezierControl>) -> Self {
        assert!(!controls.is_empty(), "Tried to construct SmoothBezierSpline with empty control points.");
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

    fn segment_and_t(&self, u: f32) -> (&CubicBezier, usize, f32) {
        // Edge case were rounding would give index error otherwise
        if u >= self.max_u {
            let i = self.segments.len() - 1;
            return (&self.segments[i], i, 1.0);
        }
        let u = u.min(self.max_u).max(0.0);
        let i = u as usize;
        (&self.segments[i], i, u.fract())
    }

    pub fn get(&self, u: f32) -> Vec2<f32> {
        let (segment, _, t) = self.segment_and_t(u);
        segment.get(t)
    }

    pub fn velocity(&self, u: f32) -> Vec2<f32> {
        let (segment, _, t) = self.segment_and_t(u);
        segment.velocity(t)
    }

    pub fn tangent(&self, u: f32) -> Vec2<f32> {
        self.velocity(u).normalized()
    }

    pub fn arc_length(&self, u: f32) -> f32 {
        let (active_segment, i, t) = self.segment_and_t(u);

        // All prior segments have the full length contribute
        let previous_length: f32 = self.segments[0..i].iter().map(|segment| segment.arc_length(1.0)).sum();

        // Arc length is prior length, plus the arc length on the active segment
        previous_length + active_segment.arc_length(t)
    }

    pub fn total_length(&self) -> f32 {
        self.arc_length(self.segments.len() as f32)
    }

    pub fn closest_point(&self, point: Vec2<f32>) -> ClosestPointOutput {

        // First inspect bounding boxes to get upper bound on distance_sq
        //
        // Store distance squared to farthest and closest corner
        let mut min_d2 = Vec::<f32>::with_capacity(self.segments.len());
        let mut upper_bound = f32::INFINITY;

        for segment in &self.segments {
            let closest_point = segment.bounding_box.closest_point(point);
            let farthest_point = segment.bounding_box.farthest_point(point);

            let closest_d2 = {
                let delta = closest_point - point;
                delta.dot(delta)
            };
            let farthest_d2 = {
                let delta = farthest_point - point;
                delta.dot(delta)
            };

            if farthest_d2 < upper_bound {
                upper_bound = farthest_d2;
            }
            min_d2.push(closest_d2)
        }

        let points = self.segments
            .iter()
            .enumerate()
            .filter_map(|(i, segment)| {
                if min_d2[i] > upper_bound {
                    None
                } else {
                    let point_output = segment.closest_point(point);
                    Some(ClosestPointOutput { 
                        parameter: i as f32 + point_output.parameter,
                        distance_sq: point_output.distance_sq
                    })
                }
            });

        points.fold(None, |accumulator, point| match accumulator {
            None => Some(point),
            Some(point_p) => match point_p.distance_sq.total_cmp(&point.distance_sq) {
                Ordering::Less | Ordering::Equal => Some(point_p),
                Ordering::Greater => Some(point)
            }
        }).expect("at least one distance to exist")
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Common Bezier curve used in tests
    fn setup_bezier() -> CubicBezier {
        let start = Vec2(-1.0, 0.0);
        let p1 = Vec2(-1.0, 1.0);
        let p2 = Vec2(1.0, 1.0);
        let end = Vec2(1.0, 0.0);
        CubicBezier::new(start, p1, p2, end)
    }

    /// Common Bezier spline used in tests
    ///
    /// S shaped
    ///
    /// x--x  x
    /// |  |  |
    /// x  x--x
    fn setup_spline() -> SmoothBezierSpline {
        SmoothBezierSpline::new(
            vec![BezierControl{ point: Vec2(0.0, 0.0), velocity: Vec2(0.0, 1.0)},
                 BezierControl{ point: Vec2(1.0, 0.0), velocity: Vec2(0.0, -1.0)},
                 BezierControl{ point: Vec2(2.0, 0.0), velocity: Vec2(0.0, 1.0)}]
        )
    }

    #[test]
    fn test_limits() {
        let bezier = setup_bezier();
        assert_eq!(bezier.get(0.0), Vec2(-1.0, 0.0));
        assert_eq!(bezier.get(1.0), Vec2(1.0, 0.0));
        assert_eq!(bezier.get(0.5).0, 0.0);  // Control point symmetric about x axis, so 0.5 should
                                             // have x=0

        let spline = setup_spline(); 
        assert_eq!(spline.get(0.0), Vec2(0.0, 0.0));
        assert_eq!(spline.get(0.5).0, 0.5);
        assert_eq!(spline.get(1.0), Vec2(1.0, 0.0));
        assert_eq!(spline.get(2.0), Vec2(2.0, 0.0));
    } 

    #[test]
    fn test_velocity() {
        let t = 0.625;  // Arbitrary

        let bezier = setup_bezier();

        // Check that velocity is consistent with finite difference approximation
        let epsilon = 0.00001;
        let c = bezier.get(t);
        let cp = bezier.get(t + epsilon);

        let v_fd = (cp - c) / epsilon;
        let v = bezier.velocity(t);

        let error = (v_fd - v).norm() / v.norm();
        assert!(error < 0.01);

        assert_eq!(bezier.velocity(0.0).0, 0.0);
        assert_eq!(bezier.velocity(1.0).0, 0.0);
        assert_eq!(bezier.velocity(0.5).1, 0.0);
    }

    #[test]
    fn test_tangent() {
        let bezier = setup_bezier();
        assert_eq!(bezier.tangent(0.0), Vec2(0.0, 1.0));
        assert_eq!(bezier.tangent(0.5), Vec2(1.0, 0.0));
        assert_eq!(bezier.tangent(1.0), Vec2(0.0, -1.0));

        let spline = setup_spline();
        assert_eq!(spline.tangent(0.0), Vec2(0.0, 1.0));
        assert_eq!(spline.tangent(0.5), Vec2(1.0, 0.0));
        assert_eq!(spline.tangent(1.0), Vec2(0.0, -1.0));
        assert_eq!(spline.tangent(2.0), Vec2(0.0, 1.0));
    }

    #[test]
    fn test_arclength() {
        // Test arc length computation on a simple straight line Bezier curve

        let spline = CubicBezier::new(Vec2(0.0, 0.0), Vec2(4.0, 3.0), Vec2(8.0, 6.0), Vec2(12.0, 9.0));
        assert_eq!(spline.arc_length(0.0), 0.0);
        assert!(spline.arc_length(1.0 / 3.0) > 4.99);
        assert!(spline.arc_length(1.0 / 3.0) < 5.01);
        assert!(spline.arc_length(2.0 / 3.0) > 9.99);
        assert!(spline.arc_length(2.0 / 3.0) < 10.01);
        assert!(spline.arc_length(1.0) > 14.99);
        assert!(spline.arc_length(1.0) < 15.01);

        let spline = SmoothBezierSpline::new(vec![BezierControl{ point: Vec2(0.0, 0.0), velocity: Vec2(4.0, 3.0)},
                                                  BezierControl{ point: Vec2(12.0, 9.0), velocity: Vec2(4.0, 3.0)},
                                                  BezierControl{ point: Vec2(24.0, 18.0), velocity: Vec2(4.0, 3.0)}]);
        assert!(spline.arc_length(1.0 / 3.0) > 4.99);
        assert!(spline.arc_length(1.0 / 3.0) < 5.01);
        assert_eq!(spline.arc_length(1.0 + 1.0 / 3.0), 20.0);
    }

    #[test]
    fn test_closest() {
        let bezier = setup_bezier();
        assert_eq!(bezier.closest_point(Vec2(-1.0, -5.0)).parameter, 0.0);
        assert_eq!(bezier.closest_point(Vec2(0.0, 7.0)).parameter, 0.5);
        assert_eq!(bezier.closest_point(Vec2(-2.0, 0.0)).distance_sq, 1.0);
        assert_eq!(bezier.closest_point(Vec2(-1.0, 0.0)), ClosestPointOutput { parameter: 0.0, distance_sq: 0.0 });

        let spline = setup_spline();
        assert_eq!(spline.closest_point(Vec2(-3.0, -7.0)).parameter, 0.0);
        assert_eq!(spline.closest_point(Vec2(0.5, 5.0)).parameter, 0.5);
        assert_eq!(spline.closest_point(Vec2(1.5, -5.0)).parameter, 1.5);
        assert_eq!(spline.closest_point(Vec2(5.0, 7.0)).parameter, 2.0);
        assert_eq!(spline.closest_point(Vec2(-4.0, -3.0)).distance_sq, 25.0);

    }

    #[test]
    fn test_bounding_box() {
        let bbox = BoundingBox::new(-1.0, 1.0, -1.0, 1.0);
        assert_eq!(bbox.closest_point(Vec2(-2.0, -2.0)), Vec2(-1.0, -1.0));
        assert_eq!(bbox.closest_point(Vec2(2.0, -2.0)), Vec2(1.0, -1.0));
        assert_eq!(bbox.closest_point(Vec2(2.0, 2.0)), Vec2(1.0, 1.0));
        assert_eq!(bbox.closest_point(Vec2(-2.0, 2.0)), Vec2(-1.0, 1.0));

        assert_eq!(bbox.closest_point(Vec2(0.3, -2.0)), Vec2(0.3, -1.0));
        assert_eq!(bbox.closest_point(Vec2(0.3, 2.0)), Vec2(0.3, 1.0));
        assert_eq!(bbox.closest_point(Vec2(-2.0, 0.2)), Vec2(-1.0, 0.2));
        assert_eq!(bbox.closest_point(Vec2(1.0, 0.2)), Vec2(1.0, 0.2));

        assert_eq!(bbox.closest_point(Vec2(0.3, 0.2)), Vec2(0.3, 0.2));

        let bezier = setup_bezier();

        let bbox = bezier._bounding_box();
        assert_eq!(bbox.min_x, -1.0);
        assert_eq!(bbox.max_x, 1.0);
        assert_eq!(bbox.min_y, 0.0);
        assert_eq!(bbox.max_y, bezier.get(0.5).1);
        assert!(bbox.max_y > 0.0);
        assert!(bbox.max_y < 1.0);
    }
}

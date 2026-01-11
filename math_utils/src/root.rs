use std::cmp::Ordering;

#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
enum Sign {
    Negative,
    Zero,
    Positive,
}

enum IntervalParity {
    Rising,
    Falling
}

pub struct FunctionObservation { 
    pub x: f32, 
    pub value: f32, 
    sign: Sign 
}

struct OpenInterval { left: FunctionObservation, right: FunctionObservation, parity: IntervalParity }

enum BisectionUpdate { 
    Root(f32),
    Interval(OpenInterval)
}


impl FunctionObservation {
    pub fn new(x: f32, value: f32) -> Self {
        let sign = match value.partial_cmp(&0.0).expect("Observation to be finite.") {
            Ordering::Equal => Sign::Zero,
            Ordering::Less => Sign::Negative,
            Ordering::Greater => Sign::Positive,
        };
        Self { x, value, sign }
    }
}


impl OpenInterval {
    fn update(self, observation: FunctionObservation) -> BisectionUpdate {
        match (self.parity, observation.sign) {
            (_, Sign::Zero) => BisectionUpdate::Root(observation.x),
            (parity @ IntervalParity::Rising, Sign::Positive) | (parity @ IntervalParity::Falling, Sign::Negative) => {
                BisectionUpdate::Interval(
                    OpenInterval{left: self.left, right: observation, parity }
                )
            },
            (parity @ IntervalParity::Rising, Sign::Negative) | (parity @ IntervalParity::Falling, Sign::Positive) => {
                BisectionUpdate::Interval(
                    OpenInterval{left: observation, right: self.right, parity }
                )
            },
        }
    }

    fn width(&self) -> f32 {
        self.right.x - self.left.x
    }
}


pub fn find_root<F>(f: F, x_min: f32, x_max: f32, width_threshold: f32) -> Option<f32>
where 
    F: Fn(f32)->f32,
{ 
    let left = FunctionObservation::new(x_min, f(x_min));
    let right = FunctionObservation::new(x_max, f(x_max));

    let parity = match (left.sign, right.sign) {
        (Sign::Zero, _) => return Some(x_min),
        (_, Sign::Zero) => return Some(x_max),
        (Sign::Negative, Sign::Positive) => IntervalParity::Rising,
        (Sign::Positive, Sign::Negative) => IntervalParity::Falling,
        (Sign::Positive, Sign::Positive) | (Sign::Negative, Sign::Negative) => return None,
    };
    let mut interval = OpenInterval { left, right, parity };
    let mut iteration: usize = 0;

    while interval.width() > width_threshold && iteration < 20 {
        let midpoint_x = 0.5*(interval.left.x + interval.right.x);
        let observation = FunctionObservation::new(midpoint_x, f(midpoint_x));
        interval = match interval.update(observation) {
            BisectionUpdate::Interval(interval) => interval,
            BisectionUpdate::Root(x) => return Some(x),
        };
        iteration += 1;
    };

    // Take the final interval and get a final estimate of the root
    //
    // Linear two-point approximation
    // g(x) = (x-a)*(f(b)-f(a)) / (b-a) + f(a)
    // 0 = (x-a)*(f(b)-f(a)) / (b-a) + f(a)
    // -f(a) = (x-a)*(f(b)-f(a)) / (b-a)
    // -f(a)*(b-a) = (x-a)*(f(b)-f(a))
    // -f(a)*(b-a) / (f(b)-f(a)) = x-a
    // a - f(a)*(b-a) / (f(b)-f(a)) = x
    let k = (interval.right.x - interval.left.x) / (interval.right.value - interval.left.value);
    Some(interval.left.x - interval.left.value * k)
}


pub fn find_local_min_differentiable<F>(fp: F, x_min: f32, x_max: f32, width_threshold: f32) -> Option<f32>
where
    F: Fn(f32) -> f32,
{
    // Evaluate the derivative at the start and end points
    let d_start = fp(x_min);
    let d_end = fp(x_max);
    if d_start > 0.0 || d_end < 0.0 {
        return None  // The passed arguments can only guarantee a local maximum
    } 

    // The derivative will have a root with negative derivative to the left and positive derivative
    // to the right. The root found will constitute a local minimum
    find_root(fp, x_min, x_max, width_threshold)
}


pub fn find_min_differentiable<F,G>(f: F, fp: G, x_min: f32, x_max: f32, width_threshold: f32) -> FunctionObservation
where
    F: Fn(f32) -> f32,
    G: Fn(f32) -> f32,
{
        let steps = 32;

        // Select a uniform grid of points and compute the value at each
        let dx = (x_max-x_min) / steps as f32;
        let values: Vec<f32> = (0 ..= steps).map(|i| f(x_min + i as f32 * dx)).collect();

        // Get the minimizing sample point
        let (i, value_i) = values.iter().enumerate()
            .fold(None, |accumulator, (i, v)| {  // We fold such that we track the distance and
                                                 // index of the smallest distance
                match accumulator {
                    None => Some((i, v)),
                    Some((i_other, v_other)) => match v.partial_cmp(v_other) {
                        None => panic!("Comparison should always work"),
                        Some(Ordering::Equal) => Some((i, v)),
                        Some(Ordering::Less) => Some((i,v)),
                        Some(Ordering::Greater) => Some((i_other, v_other)),
                    }
                }
            })
            .expect("values to have at least one element");
        let xi = x_min + i as f32 * dx;

        // Do derivative-based extremum finding around the approximate input
        let x_left = (xi-dx).max(x_min);
        let x_right = (xi+dx).min(x_max);

        if let Some(x_lm) = find_local_min_differentiable(fp, x_left, x_right, width_threshold) {
            let value_lm = f(x_lm);
            match value_lm.partial_cmp(value_i).expect("input x to be finite") {
                Ordering::Less => FunctionObservation::new(x_lm, value_lm),
                Ordering::Greater => FunctionObservation::new(xi, *value_i),
                Ordering::Equal => FunctionObservation::new(xi, *value_i),
            } 
        } else {
            FunctionObservation::new(xi, *value_i)
        }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_root() {
        // Find square root of 9
        let f = |x| x*x - 9.0;

        assert_eq!(find_root(f, 1.0, 4.0, 1e-3), Some(3.0));  // Will be found at an exact bisection
        assert_eq!(find_root(f, 0.0, 3.1415926535, 1e-3), Some(3.0));
    }

    #[test]
    fn test_min() {
        // Find minimum of cos(x)
        let f = |x: f32| x.cos();
        let fp = |x: f32| -x.sin();

        // Case when global minimum is local minimum inside the range
        let extremum = find_min_differentiable(f, fp, 3.0, 3.5, 1e-3);
        assert_eq!(extremum.x, 3.14159265358979);
        assert_eq!(extremum.value, -1.0);

        // Case when global minimum is boundary value
        let extremum = find_min_differentiable(f, fp, 0.5, 1.0, 1e-3);
        assert_eq!(extremum.x, 1.0);

        // Find minimum of x**2, check case when global minimum is a local minimum *at* boundary
        let f = |x: f32| x*x;
        let fp = |x: f32| 2.0*x;

        let extremum = find_min_differentiable(f, fp, -1.0, 0.0, 1e-3);
        assert_eq!(extremum.x, 0.0);
        assert_eq!(extremum.value, 0.0);

        // Find minimum of x³-x; check case when function has a local minimum, but global minimum
        // is at boundary
        let f = |x: f32| x*x*x - x;
        let fp = |x: f32| 3.0*x*x - 1.0;

        let extremum = find_min_differentiable(f, fp, -2.0, 2.0, 1e-3);
        assert_eq!(extremum.x, -2.0);

        // Restricting the domain, the global minimum is a local minimum at roughly 0.577
        let extremum = find_min_differentiable(f, fp, -1.0, 1.0, 1e-3);
        assert!(extremum.x < 0.58);
        assert!(extremum.x > 0.57);
    }

}


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

pub struct FunctionObservation { x: f32, value: f32, sign: Sign }


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
        return self.right.x - self.left.x
    }
}


pub fn find_root<F>(f: F, x_min: f32, x_max: f32) -> Option<f32>
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

    while interval.width() > 1e-3 && iteration < 100 {
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


pub fn find_min_differentiable<F>(fp: F, x_min: f32, x_max: f32) -> Option<f32>
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
    find_root(fp, x_min, x_max)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_root() {
        // Find square root of 9
        let f = |x| x*x - 9.0;

        assert_eq!(find_root(f, 1.0, 4.0), Some(3.0));  // Will be found at an exact bisection
        assert_eq!(find_root(f, 0.0, 3.1415926535), Some(3.0));
    }

    #[test]
    fn test_min() {
        // Find minimum of cos(x)
        let fp = |x: f32| -x.sin();

        let extremum = find_min_differentiable(fp, 3.0, 3.5);
        assert_eq!(extremum, Some(3.14159265358979));
        let extremum = find_min_differentiable(fp, -1.0, 1.0);
        assert_eq!(extremum, None);
    }

}


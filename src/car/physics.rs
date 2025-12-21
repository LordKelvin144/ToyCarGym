use crate::math_utils::Vec2;


#[derive(Debug)]
pub struct CarConfig {
    pub length: f32,
    pub front_axle: f32,
    pub back_axle: f32,
    pub max_delta: f32,
    pub acceleration: f32,
    pub brake_acceleration: f32
}

#[derive(Debug, Clone)]
pub struct CarState {
    pub position: Vec2::<f32>,
    pub unit_forward: Vec2::<f32>,
    pub speed: f32,
}

impl Default for CarConfig {
    fn default() -> Self {
        Self { length: 3.0, front_axle: 0.5, back_axle: 2.5, max_delta: 0.5, acceleration: 6.0, brake_acceleration: 8.0 }
    }
}

impl Default for CarState {
    fn default() -> Self { 
        CarState {position: Vec2(0.0, 0.0), speed: 8.0, unit_forward: Vec2(1.0, 0.0) }
    }
}

#[derive(Debug)]
pub struct CarInput {
    pub forward_acc: f32,
    pub steer_delta: f32
}

impl Default for CarInput {
    fn default() -> Self {
        CarInput { forward_acc: 0.0, steer_delta: 0.0 }
    }
}


/// Computes the reciprocal turn radius (positive when turning to the left) when having a wheel deflection
/// 'delta'
fn inv_turn_radius(config: &CarConfig, delta: f32) -> f32 {
    // L/R = tan(delta)
    // =>  1/R = tan(delta)/L
    delta.tan() / config.length
}


impl CarState {
    pub fn update(&self, input: &CarInput, dt: f32, config: &CarConfig) -> Self {
        // Current speed
        let speed = self.speed;

        // Get average speed over the time step
        let avg_speed = (speed + 0.5*dt*input.forward_acc).max(-2.0);
        let new_speed = (speed + dt*input.forward_acc).max(-2.0);

        // Determine the turning circle
        let signed_inv_radius = inv_turn_radius(config, input.steer_delta);
        let arc = avg_speed * dt;
        let signed_radians_traversed = arc * signed_inv_radius;
        let phi = signed_radians_traversed.abs();  // positive angle

        // Unit vectors
        let e_left = self.unit_forward.rotate90();

        // Compute the new position
        //
        // If the angle is small (large radius), we use small angle approximation to avoid
        // numerical instability with 1/R
        let new_position = {

            // Get forward and lateral displacements

            let (forward, left) = if phi > 1.0 {
                // Forward displacement is R*sin(r)
                //
                // Lateral displacement (absolute value) is
                // R - R*cos(r) = R*[1-cos(r)]
                let radius = 1.0 / signed_inv_radius.abs();

                (radius * phi.sin(), radius * (1.0-phi.cos()))
            } else {
                // Forward displacement is 
                // R*sin(phi) = R*sin(arc / R) = R * [arc/R - 1/6(arc/R)^3 + O(arc/R)^5]
                //     = arc - (1/6)arc^3*(1/R)^2 + O((1/R)^5)
                //     = arc * [1 - 1/6*arc^2/R^2 + O((arc/R)^4)]
                //     = arc * [1 - 1/6*(arc/R)^2 + O((arc/R)^4)]
                //     = arc * [1 - 1/6*phi^2 + O(phi^4)]]
                //
                // Lateral displacement (absolute value) is
                // R - R*cos(phi) = R*[1-cos(phi)] = R * [1 - (1- 0.5*phi^2 + O(phi^4))]
                //     = R * [0.5 * phi^2 + O(phi^4)]
                //     = 0.5 * arc^2/R + O(arc*r^3)
                //     = 0.5*arc * phi
            
                let forward_factor = 1.0 - (1.0/6.0)*phi.powi(2);
                let forward = arc*forward_factor;

                let left = 0.5*arc * signed_radians_traversed;
                (forward, left)
            };

            // Compute the vector displacement
            self.position + self.unit_forward*forward + e_left*left
        };

        // Rotate the velocity vector according to the swept arc
        let new_unit_forward = self.unit_forward.rotate(signed_radians_traversed);

        CarState { position: new_position, speed: new_speed, unit_forward: new_unit_forward }
    }
}


#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_inertial() {
        let config = CarConfig { length: 1.0, back_axle: 0.0, front_axle: 1.0, ..CarConfig::default() };
        let initial_state = CarState { position: Vec2(0.0, 0.0), speed: 1.0, unit_forward: Vec2(1.0, 0.0) };
        let input = CarInput { forward_acc: 0.0, steer_delta: 0.0 };

        let mut state = initial_state.clone();
        let dt = 1.0/16.0;
        for _ in 1 ..= 32 {
            state = state.update(&input, dt, &config);
        };
        assert_eq!(state.position, Vec2(2.0, 0.0));
    }

    #[test]
    fn test_circle() {
        let config = CarConfig { length: 1.0, back_axle: 0.0, front_axle: 1.0, ..CarConfig::default() };
        let initial_state = CarState { position: Vec2(0.0, 0.0), speed: 1.0, unit_forward: Vec2(1.0, 0.0) };

        // Deflect wheel 45 degrees
        // Turning radius is same as length = 1
        let input = CarInput { forward_acc: 0.0, steer_delta: 45.0_f32.to_radians() };  

        // Check the center of rotation
        assert_eq!(inv_turn_radius(&config, 45.0_f32.to_radians()), 1.0);

        // Drive for pi/2 units of time -> should traverse 90 degrees of the circle
        let mut state = initial_state.clone();
        let phi = 90.0_f32.to_radians();
        let dt = phi / 64.0;
        for _ in 1 ..= 64 {
            state = state.update(&input, dt, &config);
        };

        // New position after 90 degrees should be (1, 1), cince the center of the circle
        // is at (0, 1)
        assert!((state.position + Vec2(-1.0, -1.0)).norm() < 0.001);
    }

    #[test]
    fn test_acceleration() {
        let config = CarConfig { length: 1.0, back_axle: 0.0, front_axle: 1.0, ..CarConfig::default() };
        let initial_state = CarState { position: Vec2(0.0, 0.0), speed: 0.0000001, unit_forward: Vec2(1.0, 0.0) };

        // Accelerate with one unit of acceleration LT^{-2}
        let input = CarInput { forward_acc: 1.0, steer_delta: 0.0 };  

        let mut state = initial_state.clone();
        let dt = 1.0 / 64.0;
        for _ in 1 ..= 64 {
            state = state.update(&input, dt, &config);
        }

        // Displacement should be 0.5*a*t^2 = 0.5; speed should be 1.0
        assert!((state.speed - 1.0).abs() < 0.001);
        assert!((state.position + Vec2(-0.5, 0.0)).norm() < 0.001);
    }
}


use super::super::physics::{CarInput, CarConfig};
use macroquad::prelude as mq;


pub struct InputKeycodes {
    pub left: mq::KeyCode,
    pub right: mq::KeyCode,
    pub accelerate: mq::KeyCode,
    pub brake: mq::KeyCode,
}


pub struct BinaryInputDynamics {
    keycodes: InputKeycodes,
}


impl BinaryInputDynamics {
    pub fn new(keycodes: InputKeycodes) -> Self {
        Self { keycodes }
    }
}

pub struct SlidingInputDynamics {
    keycodes: InputKeycodes,
    delta_rate: f32,
}

impl SlidingInputDynamics {
    pub fn new(keycodes: InputKeycodes, delta_rate: f32) -> Self {
        Self { keycodes, delta_rate }
    }
}


// A class for reading keyboard input
pub trait CarInputDynamics {

    // Reads the current input and the time delta, and produces a new input
    fn update(&self, input: &CarInput, dt: f32, config: &CarConfig) -> CarInput;
}


impl CarInputDynamics for BinaryInputDynamics {
    fn update(&self, _input: &CarInput, _dt: f32, config: &CarConfig) -> CarInput {
        let mut steer_delta = 0.0;
        let mut forward_acc = 0.0;
        if mq::is_key_down(self.keycodes.left) {
            steer_delta += config.max_delta;
        }
        if mq::is_key_down(self.keycodes.right) {
            steer_delta -= config.max_delta;
        }
        if mq::is_key_down(self.keycodes.accelerate) {
            forward_acc += config.acceleration;
        }
        if mq::is_key_down(self.keycodes.brake) {
            forward_acc -= config.brake_acceleration;
        }
        CarInput { steer_delta, forward_acc }
    }

}


impl CarInputDynamics for SlidingInputDynamics {
    fn update(&self, input: &CarInput, dt: f32, config: &CarConfig) -> CarInput {
        let mut steer_delta = input.steer_delta;
        let mut forward_acc = 0.0;
        let mut turning = false;

        // If we have a net input, pull delta in the appropriate direction
        if mq::is_key_down(self.keycodes.left) {
            steer_delta += dt*self.delta_rate;
            turning = true;
        }
        if mq::is_key_down(self.keycodes.right) {
            steer_delta -= dt*self.delta_rate;
            turning = true;
        }

        // Clip the delta
        if steer_delta > config.max_delta {
            steer_delta = config.max_delta;
        } else if steer_delta < -config.max_delta {
            steer_delta = -config.max_delta;
        }
        
        // If we aren't turning, pull delta toward center
        if !turning {
            let step = -dt*self.delta_rate*steer_delta.signum();
            if steer_delta.abs() < step.abs() {
                steer_delta = 0.0
            } else {
                steer_delta += step;
            }
        }

        // Handle acceleration input
        if mq::is_key_down(self.keycodes.accelerate) {
            forward_acc += config.acceleration;
        }
        if mq::is_key_down(self.keycodes.brake) {
            forward_acc -= config.brake_acceleration;
        }
        CarInput { steer_delta, forward_acc }
    }
}

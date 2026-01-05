use macroquad::prelude as mq;

use car_sim::physics::{CarInput, CarConfig};


pub struct InputKeycodes {
    pub left: mq::KeyCode,
    pub right: mq::KeyCode,
    pub accelerate: mq::KeyCode,
    pub brake: mq::KeyCode,
}


pub trait CarInputSource {
    /// Get player input
    fn read(&self, config: &CarConfig) -> CarInput;
}


pub struct KeyboardInput {
    keycodes: InputKeycodes
}

impl KeyboardInput {
    pub fn new(keycodes: InputKeycodes) -> Self {
        Self { keycodes }
    }
}

impl Default for KeyboardInput {
    fn default() -> Self {
        KeyboardInput { 
            keycodes: InputKeycodes {
                left: mq::KeyCode::Left, 
                right: mq::KeyCode::Right, 
                accelerate: mq::KeyCode::Up,
                brake: mq::KeyCode::Down,
            }
        }
    }
}


impl CarInputSource for KeyboardInput {
    fn read(&self, config: &CarConfig) -> CarInput {
        let mut target_delta = 0.0;
        let mut forward_acc = 0.0;
        let mut braking = false;

        if mq::is_key_down(self.keycodes.left) {
            target_delta += config.max_delta;
        }
        if mq::is_key_down(self.keycodes.right) {
            target_delta -= config.max_delta;
        }
        if mq::is_key_down(self.keycodes.accelerate) {
            forward_acc += config.acceleration;
        }
        if mq::is_key_down(self.keycodes.brake) {
            braking = true;
        }
        
        CarInput { target_delta, forward_acc, braking }
    }
}


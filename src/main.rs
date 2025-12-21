mod env;
mod walk;
mod car;
mod math_utils;

use walk::{Walk, RandomWalk, Square, Move};
use env::{DeterministicEnv, RandomEnv};

use car::physics::{CarState, CarInput, CarConfig};
use car::game::{draw_car, 
    InputKeycodes, CarInputDynamics, BinaryInputDynamics, SlidingInputDynamics};

use macroquad::prelude as mq;
use macroquad::prelude::{KeyCode};


#[macroquad::main("Car RL")]
async fn main() {

    let config = CarConfig { 
        length: 4.0,
        max_delta: 0.6,
        acceleration: 9.8,
        brake_acceleration: 8.0
    };
    //let input_dynamics = BinaryInputDynamics::new( 
        //InputKeycodes { left: KeyCode::Left, right: KeyCode::Right, accelerate: KeyCode::Up, brake: KeyCode::Down },
    //);
    let input_dynamics = SlidingInputDynamics::new(
        InputKeycodes { left: KeyCode::Left, right: KeyCode::Right, accelerate: KeyCode::Up, brake: KeyCode::Down },
        0.7
    );
    let mut state = CarState::default();
    let mut input = CarInput::default();

    loop {
        mq::clear_background(mq::DARKPURPLE);

        let dt = mq::get_frame_time();

        // Handle user input
        input = input_dynamics.update(&input, dt, &config);
        
        // Compute physics
        // println!("dt={dt}, state={state:?}");
        // println!("input={:?}, speed={}", &input, &state.velocity.norm());
        state = state.update(&input, dt, &config);

        // Draw the car
        draw_car(&state, &config);

        mq::next_frame().await
    }
}

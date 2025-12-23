mod env;
mod walk;
mod car;
mod math_utils;

use walk::{Walk, RandomWalk, Square, Move};
use env::{DeterministicEnv, RandomEnv};

use car::physics::{CarState, CarInput, CarConfig};
use car::game::{
    draw_car, draw_map, draw_lidar,
    InputKeycodes, CarInputDynamics, BinaryInputDynamics, SlidingInputDynamics
};
use car::map::{CellMap, LidarArray};
use car::map;

use macroquad::prelude as mq;
use macroquad::prelude::{KeyCode};


#[macroquad::main("Car RL")]
async fn main() {

    // Create the race map
    let map = CellMap::new(&map::FOLD, 10.0);

    // Create a LiDAR array
    let lidar_angles: Vec<f32> = (1 ..= 45).map(|angle| 4.0*(angle as f32)).collect();
    let lidar_array = LidarArray::new(lidar_angles);

    // Set physical settings for car
    let config = CarConfig { 
        length: 4.0,
        back_axle: 0.5,
        front_axle: 3.5,
        ..Default::default()
    };

    // Create an object for managing user input
    let input_dynamics = SlidingInputDynamics::new(
        InputKeycodes { left: KeyCode::Left, right: KeyCode::Right, accelerate: KeyCode::Up, brake: KeyCode::Down },
        0.7
    );

    let mut do_draw_map = true;
    let mut do_draw_lidar = true;

    // Intialize simulator state
    let mut state = CarState::default();
    let mut input = CarInput::default();

    loop {

        let dt = mq::get_frame_time();

        // Handle user input
        input = input_dynamics.update(&input, dt, &config);
        if mq::is_key_pressed(KeyCode::Z) {
            do_draw_lidar = !do_draw_lidar;
        }
        if mq::is_key_pressed(KeyCode::M) {
            do_draw_map = !do_draw_map;
        }
        
        // Run physics
        state = state.update(&input, dt, &config);

        // Check if we have crashed
        let crashed = map.is_crashed(&state, &config);
        if crashed {
            println!("Crashed: position={:?}", state.position)
        }

        // Get LIDAR
        let readings = map.read_lidar(&state, &lidar_array);
        // println!("Lidar readings: {:?}", readings);

        // Draw
        mq::clear_background(mq::Color{ r: 0.3, g: 0.8, b: 0.4, a: 0.5 });
        if do_draw_map { draw_map(&map); }
        if do_draw_lidar { draw_lidar(&state, &lidar_array, &readings); }
        draw_car(&state, &input, &config);

        mq::next_frame().await
    }
}

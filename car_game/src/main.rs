use macroquad::prelude as mq;
use macroquad::prelude::{KeyCode};

use graphics_utils::{ScreenTransform};

use car_sim::physics::{CarState, CarConfig};
use car_sim::lidar::{LidarArray};
use car_sim::map::{Road};
use car_sim::map;

use car_game::graphics::{draw_car, draw_lidar, DrawRoad};
use car_game::input::{KeyboardInput, CarInputSource};


#[macroquad::main("Car RL")]
async fn main() {

    // Create an object tracking coordinate transformations for drawing
    let mut transform = ScreenTransform::new(10.0);

    // Create the race map
    let road = map::make_racetrack();

    // Create a LiDAR array
    let lidar_array = LidarArray::default();

    // Set physical settings for car
    let config = CarConfig::default();

    // Create an object for managing user input
    let keyboard_input = KeyboardInput::default();
    let mut do_draw_road = true;
    let mut do_draw_lidar = true;

    // Intialize simulator state
    let mut state = CarState::default();

    loop {

        let dt = mq::get_frame_time();

        // Handle user input
        let input = keyboard_input.read(&config);
        if mq::is_key_pressed(KeyCode::Z) {
            do_draw_lidar = !do_draw_lidar;
        }
        if mq::is_key_pressed(KeyCode::M) {
            do_draw_road = !do_draw_road;
        }
        
        // Run physics
        state = state.update(&input, dt, &config);

        // Check if we have crashed
        let _crashed = road.is_crashed(&state, &config);

        // Get LIDAR
        let readings = road.read_lidar(&state, &lidar_array);

        // Draw
        transform.set_center(state.position);

        mq::clear_background(mq::Color{ r: 0.3, g: 0.8, b: 0.4, a: 0.5 });
        if do_draw_road { road.draw_road(&transform); }
        if do_draw_lidar { draw_lidar(&state, &lidar_array, &readings, &transform); }
        draw_car(&state, &config, &transform);

        mq::next_frame().await
    }
}

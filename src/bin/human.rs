use car_rl::car::physics::{CarState, CarInput, CarConfig};
use car_rl::car::game::{
    draw_car, draw_lidar,
    InputKeycodes, CarInputDynamics, SlidingInputDynamics
};
use car_rl::car::map::{LidarArray, Road, DrawRoad};
use car_rl::car::map;

use macroquad::prelude as mq;
use macroquad::prelude::{KeyCode};

use car_rl::graphics_utils::{ScreenTransform};


#[macroquad::main("Car RL")]
async fn main() {

    // Create an object tracking coordinate transformations for drawing
    let mut transform = ScreenTransform::new(10.0);

    // Create the race map
    let road = map::make_oval();

    // Create a LiDAR array
    let lidar_angles: Vec<f32> = (1 ..= 22).map(|angle| 8.0*(angle as f32)).collect();
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

    let mut do_draw_road = true;
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
            do_draw_road = !do_draw_road;
        }
        
        // Run physics
        state = state.update(&input, dt, &config);

        // Check if we have crashed
        let crashed = road.is_crashed(&state, &config);
        if crashed {
            println!("Crashed: position={:?}", state.position)
        }

        // Get LIDAR
        let readings = road.read_lidar(&state, &lidar_array);

        // Draw
        transform.set_center(state.position);

        mq::clear_background(mq::Color{ r: 0.3, g: 0.8, b: 0.4, a: 0.5 });
        if do_draw_road { road.draw_road(&transform); }
        if do_draw_lidar { draw_lidar(&state, &lidar_array, &readings, &transform); }
        draw_car(&state, &input, &config, &transform);

        mq::next_frame().await
    }
}

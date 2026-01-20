use math_utils::Vec2;
use crate::physics::{CarState, CarConfig};
use crate::lidar::LidarArray;


/// A trait representing a representation of a road in the game
/// Should support a method of determining whether a car is crashed, and methods for determining
/// lidar stats
pub trait Road {
    fn is_crashed(&self, state: &CarState, config: &CarConfig) -> bool;
    fn ray_collision(&self, point: Vec2, direction: Vec2) -> Vec2;
    fn read_lidar(&self, state: &CarState, lidar: &LidarArray) -> Vec<f32> {
        lidar.get_angles()
            .iter()
            .map(|&angle| {
                let direction = state.unit_forward.rotate(angle);
                let intersection = self.ray_collision(state.position, direction);
                // Get distance = projection along 'direction'
                direction.dot(intersection-state.position)
            })
            .collect()
    }
}


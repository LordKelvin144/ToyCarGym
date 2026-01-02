use crate::car::physics::{CarState, CarConfig};
use super::lidar::LidarArray;
use crate::math_utils::Vec2;


pub trait Road {
    fn is_crashed(&self, state: &CarState, config: &CarConfig) -> bool;
    fn ray_collision(&self, point: Vec2::<f32>, direction: Vec2::<f32>) -> Vec2::<f32>;
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


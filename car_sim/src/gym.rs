use crate::physics::{CarState, CarInput, CarConfig};
use crate::map::{Road, SplineMap};
use crate::lidar::LidarArray;
use math_utils::spline::ClosestPointOutput;


#[repr(u8)]
pub enum Action {
    Left = 0,
    Right = 1,
    Accelerate = 2,
    Brake = 3
}


#[derive(Debug)]
pub struct Observation {
    pub lidar_readings: Vec<f32>,
    pub reward: f32,
    pub done: bool
}


#[derive(Debug)]
pub struct ResetObservation {
    pub lidar_readings: Vec<f32>
}


#[derive(Debug)]
pub struct RewardConfig {
    pub travel_coeff: f32,
    pub center_coeff: f32,
    pub crash_reward: f32,
}

impl Default for RewardConfig {
    fn default() -> Self {
        Self { travel_coeff: 1.0, center_coeff: 0.5, crash_reward: -100.0 }
    }
}

#[derive(Debug)]
pub struct SimConfig {
    pub car: CarConfig,
    pub reward: RewardConfig,
    pub lidar: LidarArray,
    pub dt: f32,
}

impl Default for SimConfig {
    fn default() -> Self {
        Self { 
            car: CarConfig::default(),
            reward: RewardConfig::default(),
            lidar: LidarArray::default(),
            dt: 0.2
        }
    }
}



pub trait Gym<A,Ro,O> {
    fn reset(&mut self) -> Ro;
    fn step(&mut self, action: A) -> O;
}


pub struct Simulator<R>
{
    pub config: SimConfig,
    pub road: R,
    pub state: CarState,
}



impl Gym<Action,ResetObservation,Observation> for Simulator<SplineMap> {
    fn reset(&mut self) -> ResetObservation {
        // self.state = CarState::default();
        let lidar_readings = self.road.read_lidar(&self.state, &self.config.lidar);
        ResetObservation { lidar_readings }
    }

    fn step(&mut self, action: Action) -> Observation {
        let SimConfig { dt, car: car_cfg, lidar, .. } = &self.config;
        let dt = *dt;

        let input = match action {
            Action::Left => CarInput { forward_acc: 0.0, target_delta: car_cfg.max_delta, braking: false },
            Action::Right => CarInput { forward_acc: 0.0, target_delta: -car_cfg.max_delta, braking: false },
            Action::Accelerate => CarInput { forward_acc: car_cfg.acceleration, target_delta: 0.0, braking: false },
            Action::Brake => CarInput { forward_acc: 0.0, target_delta: 0.0, braking: true },
        };
        let new_state = self.state.update(&input, dt, car_cfg);

        let lidar_readings = self.road.read_lidar(&new_state, lidar);
        let is_crashed = self.road.is_crashed(&new_state, car_cfg);

        let reward = self.reward(&self.state, &new_state, is_crashed);

        let done = is_crashed;

        Observation { lidar_readings, reward, done }
    }

}

impl Simulator<SplineMap> {
    pub fn new(config: SimConfig, road: SplineMap) -> Self {
        let state = CarState::default();

        Self { config, road, state }
    }

    fn reward(&self, state: &CarState, new_state: &CarState, is_crashed: bool) -> f32 {
        let rcfg = &self.config.reward;

        let ClosestPointOutput { parameter: p1, distance_sq: d1_sq } = self.road.spline.closest_point(state.position);
        let ClosestPointOutput { parameter: p2, distance_sq: d2_sq } = self.road.spline.closest_point(new_state.position);
        let travel1 = self.road.spline.arc_length(p1);
        let travel2 = self.road.spline.arc_length(p2);

        let travel = (travel2 - travel1) % self.road.spline.total_length();
        let d_sq_decrease = d2_sq - d1_sq;
        rcfg.travel_coeff * travel 
            + rcfg.center_coeff * d_sq_decrease 
            + rcfg.crash_reward*(is_crashed as i32 as f32)
    }
}


#[cfg(test)]
mod tests {
    use super::*;
    use crate::map;

    fn make_sim() -> Simulator<SplineMap> {
        let config = SimConfig::default();
        let road = map::make_oval();
        Simulator::new(config, road)
    }

    #[test]
    fn test_run() {
        let mut env = make_sim();
        let _reset_obs = env.reset();
        for _ in 1 .. 5 {
            let _observation = env.step(Action::Accelerate);
        }
        for _ in 1 .. 10 {
            let _observation = env.step(Action::Brake);
        }
        for _ in 1 .. 5 {
            let _observation = env.step(Action::Left);
            let _observation = env.step(Action::Right);
        }

    }
}


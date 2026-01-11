use crate::physics::{CarState, CarInput, CarConfig};
use crate::map::{Road, SplineMap};
use crate::lidar::LidarArray;
use math_utils::spline::ClosestPointOutput;


#[repr(u8)]
pub enum Action {
    Left = 0,
    Right = 1,
    Accelerate = 2,
    Brake = 3,
    Coast = 4,
}

pub struct InvalidActionError;

impl TryFrom<u8> for Action {
    type Error = InvalidActionError;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            x if x == Action::Left as u8 => Ok(Action::Left),
            x if x == Action::Right as u8 => Ok(Action::Right),
            x if x == Action::Accelerate as u8 => Ok(Action::Accelerate),
            x if x == Action::Brake as u8 => Ok(Action::Brake),
            x if x == Action::Coast as u8 => Ok(Action::Coast),
            _ => Err(InvalidActionError)
        }
    }
}


#[derive(Debug)]
pub struct TransitionObservation {
    pub reward: f32,
    pub done: bool
}


#[derive(Debug)]
pub struct StateObservation {
    pub lidar_readings: Vec<f32>
}


#[derive(Debug)]
pub struct RewardConfig {
    pub travel_coeff: f32,
    pub center_coeff: f32,
    pub crash_reward: f32,
    pub center_integral_coeff: f32,
}

impl Default for RewardConfig {
    fn default() -> Self {
        Self { 
            travel_coeff: 1.0, center_coeff: 2.0, crash_reward: -100.0,
            center_integral_coeff: 1.0
        }
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



pub struct Simulator<R>
{
    pub config: SimConfig,
    pub road: R,
    pub state: CarState,
    t: f32,
    i: usize,
}



impl Simulator<SplineMap> {
    pub fn reset(&mut self) {
        self.state = CarState::default();
        self.t = 0.0;
        self.i = 0;
    }

    pub fn step(&mut self, action: Action) -> TransitionObservation {
        let SimConfig { dt, car: car_cfg, .. } = &self.config;
        let dt = *dt;

        let input = match action {
            Action::Left => CarInput { forward_acc: 0.0, target_delta: car_cfg.max_delta, braking: false },
            Action::Right => CarInput { forward_acc: 0.0, target_delta: -car_cfg.max_delta, braking: false },
            Action::Accelerate => CarInput { forward_acc: car_cfg.acceleration, target_delta: 0.0, braking: false },
            Action::Brake => CarInput { forward_acc: 0.0, target_delta: 0.0, braking: true },
            Action::Coast => CarInput { forward_acc: 0.0, target_delta: 0.0, braking: false },
        };
        let new_state = self.state.update(&input, dt, car_cfg);

        let is_crashed = self.road.is_crashed(&new_state, car_cfg);

        let reward = self.reward(&self.state, &new_state, is_crashed);

        let done = is_crashed;

        // Do the transition
        self.state = new_state;
        self.t += dt;
        self.i += 1;

        TransitionObservation { reward, done }
    }

    pub fn observe(&self) -> StateObservation {
        let lidar_readings = self.road.read_lidar(&self.state, &self.config.lidar);
        StateObservation { lidar_readings }
    }

}

impl Simulator<SplineMap> {
    pub fn new(config: SimConfig, road: SplineMap) -> Self {
        let state = CarState::default();

        Self { config, road, state, t: 0.0, i: 0 }
    }

    fn reward(&self, state: &CarState, new_state: &CarState, is_crashed: bool) -> f32 {
        let rcfg = &self.config.reward;

        let ClosestPointOutput { parameter: p1, distance_sq: d1_sq } = self.road.spline.closest_point(state.position);
        let ClosestPointOutput { parameter: p2, distance_sq: d2_sq } = self.road.spline.closest_point(new_state.position);
        let travel1 = self.road.spline.arc_length(p1);
        let travel2 = self.road.spline.arc_length(p2);

        let total_length = self.road.spline.total_length();
        let travel = (travel2 - travel1 + 1.5*total_length) % total_length - 0.5*total_length;
        let d_sq_decrease = d2_sq - d1_sq;
        rcfg.travel_coeff * travel 
            + rcfg.center_coeff * d_sq_decrease 
            - rcfg.center_integral_coeff * d2_sq * self.config.dt
            + rcfg.crash_reward*(is_crashed as i32 as f32)
    }

    /// Get the clock of the simulator
    pub fn get_t(&self) -> f32 {
        self.t
    }

    /// Get the iteration that the simulator is at
    pub fn get_i(&self) -> usize {
        self.i
    }
}


#[cfg(test)]
mod tests {
    use super::*;
    use crate::map;

    fn make_sim() -> Simulator<SplineMap> {
        let config = SimConfig { dt: 0.25, ..SimConfig::default() };
        let road = map::make_oval();
        Simulator::new(config, road)
    }

    #[test]
    fn test_stable() {
        let mut env = make_sim();
        env.reset();
        let _observation = env.step(Action::Accelerate);
        let _observation = env.step(Action::Brake);
        let _observation = env.step(Action::Left);
        let _observation = env.step(Action::Right);
        assert_eq!(env.get_i(), 4);
        assert_eq!(env.get_t(), 4.0*env.config.dt)
    }

    #[test]
    fn test_crash() {
        let mut env = make_sim();
        let mut done = false;
        let mut reward = 0.0;

        env.reset();

        // Accelerate uncontrollably; should crash eventually
        for _ in 1 .. 50 {
            TransitionObservation { done, reward } = env.step(Action::Accelerate);
            dbg!(reward, done);
            if done {
                break
            }
        }
        assert!(done);
        assert!(reward < 0.0)
    }
}


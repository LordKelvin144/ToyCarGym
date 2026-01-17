use pyo3::prelude::*;
use pyo3::exceptions::PyValueError;
use numpy::PyArray1;


use car_sim::map;
use car_sim::gym;

mod graphics;
use graphics::{SplineRoadExport, CarGraphicsExport};


#[pyclass(module="gym_car")]
struct RacingEnv {
    sim: gym::Simulator<map::SplineMap>,
    observe_delta: bool,
    observe_speed: bool,
}


#[pymethods]
impl RacingEnv {
    #[new]
    #[pyo3(
        signature = (dt=None, crash_reward=None, travel_coeff=None, center_coeff=None, center_integral_coeff=None, observe_delta=true, observe_speed=true, seed=None)
    )]
    fn new(
        dt: Option<f32>,
        crash_reward: Option<f32>,
        travel_coeff: Option<f32>,
        center_coeff: Option<f32>,
        center_integral_coeff: Option<f32>,
        observe_delta: bool,
        observe_speed: bool,
        seed: Option<u64>,
    ) -> Self {
        let mut config = gym::SimConfig::default();
        if let Some(dt) = dt {
            config.dt = dt;
        }
        if let Some(crash_reward) = crash_reward {
            config.reward.crash_reward = crash_reward;
        }
        if let Some(travel_coeff) = travel_coeff {
            config.reward.travel_coeff = travel_coeff;
        }
        if let Some(center_coeff) = center_coeff {
            config.reward.center_coeff = center_coeff;
        }
        if let Some(center_integral_coeff) = center_integral_coeff {
            config.reward.center_integral_coeff = center_integral_coeff;
        }

        let road = map::make_racetrack();
        let mut this = Self { sim: gym::Simulator::new(config, road, seed), observe_delta, observe_speed };
        this.reset(None);
        this
    }

    #[pyo3( signature = (seed=None) )]
    fn reset(&mut self, seed: Option<u64>) {
        self.sim.reset(seed)
    }

    fn step(&mut self, action: u8) -> PyResult<(f32, bool)> {
        let action = gym::Action::try_from(action)
            .map_err(|_| PyValueError::new_err(
                    format!("Invalid action value '{}'. Action must be integer between 0 and 4.", action)
                )
            )?;

        let gym::TransitionObservation { reward, done } = self.sim.step(action);
        
        Ok((reward, done))
    }

    fn observe<'py>(&self, py: Python<'py>) -> Py<PyArray1<f32>> {
        let gym::StateObservation { lidar_readings, steer_delta, speed } = self.sim.observe();
        let mut data = lidar_readings;
        if self.observe_delta {
            data.push(steer_delta);
        }
        if self.observe_speed {
            data.push(speed);
        }

        PyArray1::from_vec(py, data).unbind()
    }

    fn export_road(&self, n_segments: usize) -> SplineRoadExport {
        graphics::export_spline_road(&self.sim.road, n_segments)
    }

    fn graphics_state(&self) -> CarGraphicsExport {
        graphics::export_car_graphics(&self.sim.state, &self.sim.config.car, &self.sim.config.lidar, &self.sim.observe().lidar_readings)
    }

    #[getter]
    fn dt(&self) -> f32 {
        self.sim.config.dt
    }

    #[getter]
    fn t(&self) -> f32 {
        self.sim.get_t()
    }

    #[getter]
    fn i(&self) -> usize {
        self.sim.get_i()
    }

    #[getter]
    fn observation_dim(&self) -> usize {
        self.sim.config.lidar.n_angles() + self.observe_delta as usize + self.observe_speed as usize
    }
}


/// A Python module implemented in Rust.
#[pymodule]
mod gym_car {
    #[pymodule_export]
    use super::RacingEnv;

    #[pymodule_export]
    use super::SplineRoadExport;

    #[pymodule_export]
    use super::CarGraphicsExport;
}

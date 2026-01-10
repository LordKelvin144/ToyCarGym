use pyo3::prelude::*;
use pyo3::exceptions::PyValueError;
use numpy::PyArray1;


use car_sim::map;
use car_sim::gym;


#[pyclass(module="gym_car")]
struct RacingEnv {
    sim: gym::Simulator<map::SplineMap>
}


#[pymethods]
impl RacingEnv {
    #[new]
    fn new() -> Self {
        let config = gym::SimConfig::default();
        let road = map::make_oval();
        Self { sim: gym::Simulator::new(config, road) }
    }

    fn reset(&mut self) {
        self.sim.reset()
    }

    fn step(&mut self, action: u8) -> PyResult<(f32, bool)> {
        let action = gym::Action::try_from(action)
            .map_err(|_| PyValueError::new_err(
                    format!("Invalid action value '{}'. Action must be integer between 0 and 3.", action)
                )
            )?;

        let gym::TransitionObservation { reward, done } = self.sim.step(action);
        
        Ok((reward, done))
    }

    fn observe<'py>(&self, py: Python<'py>) -> Py<PyArray1<f32>> {
        let gym::StateObservation { lidar_readings } = self.sim.observe();
        PyArray1::from_vec(py, lidar_readings).unbind()
    }


}


/// A Python module implemented in Rust.
#[pymodule]
mod gym_car {
    use pyo3::prelude::*;

    #[pymodule_export]
    use super::RacingEnv;

    /// Formats the sum of two numbers as string.
    #[pyfunction]
    fn sum_as_string(a: usize, b: usize) -> PyResult<String> {
        Ok((a + b).to_string())
    }
}

use pyo3::prelude::pyclass;
use math_utils::Vec2;

use car_sim::physics::{CarState, CarConfig};
use car_sim::lidar::LidarArray;
use car_sim::map::SplineMap;


#[pyclass(module="gym_car")]
pub struct SplineRoadExport {
    #[pyo3(get)]
    pub left_x: Vec<f32>,
    #[pyo3(get)]
    pub left_y: Vec<f32>,
    #[pyo3(get)]
    pub right_x: Vec<f32>,
    #[pyo3(get)]
    pub right_y: Vec<f32>,
}



struct SplineRoadExporter {
    left: Vec<Vec2>,
    right: Vec<Vec2>,
}


impl SplineRoadExporter {
    fn new() -> Self {
        Self {
            left: Vec::new(),
            right: Vec::new(),
        }
    }

    fn push(&mut self, left: Vec2, right: Vec2) {
        self.left.push(left);
        self.right.push(right);
    }

    fn export(&self) -> SplineRoadExport {
        SplineRoadExport {
            left_x: self.left.iter().map(|vec| vec.0).collect(),
            left_y: self.left.iter().map(|vec| vec.1).collect(),
            right_x: self.right.iter().map(|vec| vec.0).collect(),
            right_y: self.right.iter().map(|vec| vec.1).collect(),
        }
    }
}

pub fn export_spline_road(road: &SplineMap, n_segments: usize) -> SplineRoadExport {
    let spline = &road.spline;
    let mut exporter = SplineRoadExporter::new();

    let max_u = spline.max_u;
    let ds = spline.total_length() / n_segments as f32;

    let mut u = 0.0;

    while u < max_u {
        let v0 = spline.velocity(u);
        let du = ds / v0.norm();
        let v = v0*0.5 + spline.velocity(u + du)*0.5;
        let du = ds / v.norm();

        let center = spline.get(u);
        let lateral = spline.tangent(u).rotate90()*0.5*road.width;
        let left = center + lateral;
        let right = center - lateral;
        exporter.push(left, right);

        u += du;
    }

    exporter.export()
}


#[pyclass(module="gym_car")]
pub struct CarGraphicsExport {
    #[pyo3(get)]
    car_x: (f32, f32, f32, f32),
    #[pyo3(get)]
    car_y: (f32, f32, f32, f32),
    #[pyo3(get)]
    lidar_center: (f32, f32),
    #[pyo3(get)]
    lidar_x: Vec<f32>,
    #[pyo3(get)]
    lidar_y: Vec<f32>,
}

const WIDTH_RATIO: f32 = 0.4;


pub fn export_car_graphics(state: &CarState, config: &CarConfig, lidar: &LidarArray, lidar_readings: &[f32]) -> CarGraphicsExport {

    // Compute all relevant points for LiDAR
    let lidar_position = state.position;

    let lidar_points = lidar.get_angles().into_iter().zip(lidar_readings)
        .map(|(&angle, &reading)| {
            let direction = state.unit_forward.rotate(angle);
            lidar_position + direction*reading
        });
    
    let mut lidar_x = Vec::<f32>::new();
    let mut lidar_y = Vec::<f32>::new();

    for Vec2(x,y) in lidar_points {
        lidar_x.push(x);
        lidar_y.push(y);
    }
    let lidar_center = {
        let Vec2(x, y) = state.position;
        (x, y)
    };

    // Compute relevant points for car
    let position = state.position;
    let back_center = position - state.unit_forward*config.back_axle;
    let half_lateral = state.unit_forward.rotate90()*config.length*WIDTH_RATIO*0.5;
    let forward_displacement = state.unit_forward*config.length;
    let back_left = back_center + half_lateral;
    let back_right = back_center - half_lateral;
    let front_left = back_left + forward_displacement;
    let front_right = back_right + forward_displacement;

    CarGraphicsExport {
        car_x: (back_left.0, front_left.0, front_right.0, back_right.0),
        car_y: (back_left.1, front_left.1, front_right.1, back_right.1),
        lidar_center,
        lidar_x,
        lidar_y,
    }
}

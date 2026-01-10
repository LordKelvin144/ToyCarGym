use pyo3::prelude::pyclass;
use math_utils::Vec2;

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
    left: Vec<Vec2<f32>>,
    right: Vec<Vec2<f32>>,
}


impl SplineRoadExporter {
    fn new() -> Self {
        Self {
            left: Vec::new(),
            right: Vec::new(),
        }
    }

    fn push(&mut self, left: Vec2<f32>, right: Vec2<f32>) {
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


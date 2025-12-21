use std::collections::HashMap;
use crate::math_utils::Vec2;

use super::physics::{CarState, CarConfig};


#[derive(Hash, PartialEq, Eq, Debug, Copy, Clone)]
pub struct Cell(pub i32, pub i32);


pub struct CellMap {
    pub cells: &'static [Cell],
    pub cell_size: f32,
    idx_map: HashMap<Cell, usize>
}


impl CellMap {
    pub fn new(cells: &'static [Cell], cell_size: f32) -> Self {
        let mut idx_map = HashMap::new();
        for (idx, &cell) in cells.iter().enumerate() {
            idx_map.insert(cell, idx);
        }
        Self { cells, cell_size, idx_map}
    }

    pub fn cell(&self, p1: Vec2::<f32>) -> Cell {
        let cell_float_vec = p1 / self.cell_size;
        Cell(cell_float_vec.0.round() as i32, cell_float_vec.1.round() as i32)
    }

    pub fn is_crashed(&self, state: &CarState, config: &CarConfig) -> bool {
        let back_point = state.position - state.unit_forward*config.back_axle;
        let front_point = back_point + state.unit_forward*config.length;
        return self.step_is_along(back_point, front_point)
    }

    fn step_is_along(&self, p1: Vec2::<f32>, p2: Vec2::<f32>) -> bool {
        let c1 = self.cell(p1);
        let c2 = self.cell(p2);
        let Some(&idx1) = self.idx_map.get(&c1) else {return true;};
        let Some(&idx2) = self.idx_map.get(&c2) else {return true;};

        // If player remains in the same squae, we have not crashed
        if idx1 == idx2 {
            return false;
        }

        // If the player moved between adjacent squares, we have not crashed
        if idx2 == idx1 + 1 || idx1 == idx2 + 1{
            false
        } else if (idx1, idx2) == (0, self.cells.len()-1) {
            false
        } else if (idx1, idx2) == (self.cells.len()-1, 0) {
            false
        } else {
            true
        }
    }
}


pub static CIRCUIT: [Cell; 8] = [Cell(0,0), Cell(1,0), Cell(2,0), Cell(2,1), Cell(2,2), Cell(1,2), Cell(0,2), Cell(0,1)];
pub static FOLD: [Cell; 8] = [Cell(0,0), Cell(1,0), Cell(2,0), Cell(2,1), Cell(1,1), Cell(1,2), Cell(0,2), Cell(0,1)];


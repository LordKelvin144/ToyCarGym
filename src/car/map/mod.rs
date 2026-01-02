mod cell_map;
mod lidar;

pub use cell_map::{Cell, CellMap};
pub use lidar::LidarArray;

pub static CIRCUIT: [Cell; 8] = [Cell(0,0), Cell(1,0), Cell(2,0), Cell(2,1), Cell(2,2), Cell(1,2), Cell(0,2), Cell(0,1)];
pub static FOLD: [Cell; 8] = [Cell(0,0), Cell(1,0), Cell(2,0), Cell(2,1), Cell(1,1), Cell(1,2), Cell(0,2), Cell(0,1)];


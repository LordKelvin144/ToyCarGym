mod cell_map;
mod spline_map;
mod traits;

pub use cell_map::{Cell, CellMap};
pub use spline_map::{SplineMap, make_oval, make_racetrack};
pub use traits::{Road};

pub static CIRCUIT: [Cell; 8] = [Cell(0,0), Cell(1,0), Cell(2,0), Cell(2,1), Cell(2,2), Cell(1,2), Cell(0,2), Cell(0,1)];
pub static FOLD: [Cell; 8] = [Cell(0,0), Cell(1,0), Cell(2,0), Cell(2,1), Cell(1,1), Cell(1,2), Cell(0,2), Cell(0,1)];


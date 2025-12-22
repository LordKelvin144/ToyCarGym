use std::collections::HashMap;
use crate::math_utils::Vec2;
use itertools::Itertools;

use super::physics::{CarState, CarConfig};


#[derive(Hash, PartialEq, Eq, Debug, Copy, Clone)]
pub struct Cell(pub i32, pub i32);


#[derive(Debug, Clone)]
enum LidarDistance {
    Specific(f32),
    Far
}

impl Ord for LidarDistance {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        match (self, other) {
            (Self::Far, Self::Far) => panic!("Cannot compar two instances of 'far'"),
            (Self::Specific(_), Self::Far) => std::cmp::Ordering::Less,
            (Self::Far, Self::Specific(_)) => std::cmp::Ordering::Greater,
            (Self::Specific(t1), Self::Specific(t2)) => {
                assert!(t1.is_finite());
                assert!(t2.is_finite());
                if t1 < t2 { 
                    std::cmp::Ordering::Less 
                } else if t1 > t2 { 
                    std::cmp::Ordering::Greater
                } else if t1 == t2 {
                    std::cmp::Ordering::Equal
                } else {panic!("Unexpected logic branch");}
            }

        }
    }
}

impl PartialOrd for LidarDistance {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}


impl PartialEq for LidarDistance {
    fn eq(&self, other: &Self) -> bool {
        match self.cmp(other) {
            std::cmp::Ordering::Equal => true,
            _ => false
        }
    }
}

impl Eq for LidarDistance {}


pub struct CellMap {
    pub cells: &'static [Cell],
    pub cell_size: f32,
    min_x: i32, 
    max_x: i32,
    min_y: i32, 
    max_y: i32,
    idx_map: HashMap<Cell, usize>
}


impl CellMap {
    pub fn new(cells: &'static [Cell], cell_size: f32) -> Self {
        let mut idx_map = HashMap::new();
        for (idx, &cell) in cells.iter().enumerate() {
            idx_map.insert(cell, idx);
        }

        let min_x = cells.iter().map(|cell| cell.0).min().expect("at least one cell");
        let max_x = cells.iter().map(|cell| cell.0).max().expect("at least one cell");
        let min_y = cells.iter().map(|cell| cell.1).min().expect("at least one cell");
        let max_y = cells.iter().map(|cell| cell.1).max().expect("at least one cell");
        Self { cells, cell_size, idx_map, min_x, max_x, min_y, max_y}
    }

    pub fn cell(&self, p1: Vec2::<f32>) -> Cell {
        let cell_float_vec = p1 / self.cell_size;
        Cell(cell_float_vec.0.round() as i32, cell_float_vec.1.round() as i32)
    }

    fn cell_idx(&self, p1: Vec2::<f32>) -> Option<usize> {
        let cell = self.cell(p1);
        self.idx_map.get(&cell).copied()
    }

    pub fn is_crashed(&self, state: &CarState, config: &CarConfig) -> bool {
        let back_point = state.position - state.unit_forward*config.back_axle;
        let front_point = back_point + state.unit_forward*config.length;
        return !self.step_is_along(back_point, front_point)
    }

    fn step_is_along(&self, p1: Vec2::<f32>, p2: Vec2::<f32>) -> bool {
        let c1 = self.cell(p1);
        let c2 = self.cell(p2);
        self.contiguous_cells(Some(&c1), Some(&c2))
    }

    fn contiguous_cells(&self, cell1: Option<&Cell>, cell2: Option<&Cell>) -> bool {
        let Some(c1) = cell1 else {return false;};
        let Some(c2) = cell2 else {return false;};
        let idx1 = self.idx_map.get(c1);
        let idx2 = self.idx_map.get(c2);
        self.contiguous_idx(idx1.copied(), idx2.copied())
    }

    fn contiguous_idx(&self, idx1: Option<usize>, idx2: Option<usize>) -> bool {
        let Some(idx1) = idx1 else {return false;};
        let Some(idx2) = idx2 else {return false;};

        // If player remains in the same square, we have not crashed
        if idx1 == idx2 {
            return true;
        }

        // If the player moved between adjacent squares, we have not crashed
        if idx2 == idx1 + 1 || idx1 == idx2 + 1{
            true
        } else if (idx1, idx2) == (0, self.cells.len()-1) {
            true
        } else if (idx1, idx2) == (self.cells.len()-1, 0) {
            true
        } else {
            false
        }
    }

    // Takes in a point and (non-normalized) direction defining a ray,
    // and finds the first intersection with the edge of the track.
    pub fn ray_intersect_edge(&self, point: Vec2::<f32>, direction: Vec2::<f32>) -> Vec2::<f32> {  
        // TODO: Remove pub
        //
        // p + t*d = (x, n)
        // p.y + t*d.y = n
        // t = (n - p.y) / d.y  
        // Sort in ascending order of t

        let norm_point = point / self.cell_size;
        let Cell(cell_x, cell_y) = self.cell(point);

        let delta_x = direction.0.signum();
        let delta_y = direction.1.signum();

        let t_x = {
            
            let n_intersections: i32 = if direction.0 > 0.0 {
                self.max_x - cell_x + 1
            } else if direction.0 < 0.0 {
                cell_x - self.min_x + 1
            } else {
                0
            };
            
            let ns = (0 .. n_intersections).map(|i| {
                let i = i as f32;
                let cell_x = cell_x as f32;
                cell_x + (i+0.5)*delta_x
        
            });
            ns.map(|n| LidarDistance::Specific((n-norm_point.0) / direction.0))
        };
        let t_y = {
            let n_intersections: i32 = if direction.1 > 0.0 {
                self.max_y - cell_y + 1
            } else if direction.1 < 0.0 {
                cell_y - self.min_y + 1
            } else {
                0
            };

            let ns = (0 .. n_intersections).map(|i| {
                let i = i as f32;
                let cell_y = cell_y as f32;
                cell_y + (i+0.5)*delta_y
            });
            ns.map(|n| LidarDistance::Specific((n-norm_point.1) / direction.1))
        };
        // Merge t_x and t_y in sorted order.
        // Also prepend a 0 and append 'Far' representing infinity.
        // the ranges between any pair of t values defines a contiguous region belonging to the
        // same cell.
        //
        // ts = [0, t1, t2, t3, ..., tN, infty]
        let ts = std::iter::once(LidarDistance::Specific(0.0))
            .chain(t_x.merge(t_y))
            .chain(std::iter::once(LidarDistance::Far));  // [0, t1, ...]

        // Loop over contiguous pairs, and retrieve the relevant cell they belong to
        // In case of a pair of specific distances, we arbitrarily use their midpoint
        // In case of (tN, infinity), we select the cell be None
        // We also report the t value at the end of the window (which the cell is associated with)

        let cell_idx = ts
            .tuple_windows()
            .map(|(t1d, t2d): (LidarDistance, LidarDistance)| -> (Option<usize>, LidarDistance) {
                let LidarDistance::Specific(t1) = t1d else {panic!("First t should be a specific distance, never Far");};
                let cell: Option<usize> = match t2d {
                    LidarDistance::Specific(t2) => {
                        let midpoint = point + direction*self.cell_size*0.5*(t1+t2);
                        self.cell_idx(midpoint)
                    },
                    LidarDistance::Far => None
                };
                (cell, t1d)
            });

        // Find the first comparison that fails
        let t_max = cell_idx.tuple_windows()
            .filter_map(|((cell_before, t_before), (cell_after, t_between)): ((Option<usize>, LidarDistance), (Option<usize>, LidarDistance))| {
                let contiguous = self.contiguous_idx(cell_before, cell_after);
                match (cell_before, contiguous) {
                    (Some(_), false) => Some(t_between),  // Had a valid cell before, so t_between
                                                           // is failure
                    (None, false) => Some(t_before),  // First cell was already problematic, so t is
                                                       // at most t_before
                    (_, true) => None  // This transition is fine
                }
            })
            .min();  // Retrieve earlies failing transition

        let Some(t_max) = t_max else {
            // If there were no comparisons made, for example if we are outside the map.
            // The intersection is then our current point.
            return point
        };

        match t_max  {
            LidarDistance::Specific(t) => point + direction * self.cell_size * t,
            LidarDistance::Far => panic!("The max distance must be concrete.")
        }
    }
}


pub static CIRCUIT: [Cell; 8] = [Cell(0,0), Cell(1,0), Cell(2,0), Cell(2,1), Cell(2,2), Cell(1,2), Cell(0,2), Cell(0,1)];
pub static FOLD: [Cell; 8] = [Cell(0,0), Cell(1,0), Cell(2,0), Cell(2,1), Cell(1,1), Cell(1,2), Cell(0,2), Cell(0,1)];


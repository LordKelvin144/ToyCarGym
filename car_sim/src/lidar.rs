use std::cmp::Ordering;


// A struct for maintaining the angles of an array of LIDAR sensors
#[derive(Debug)]
pub struct LidarArray {
    angles: Vec<f32>
}


impl LidarArray {
    pub fn new(angles: Vec<f32>) -> Self {
        let angles = angles.clone().into_iter().rev().map(|angle| -angle)
            .chain(std::iter::once(0.0))
            .chain(angles)
            .map(|angle| angle.to_radians())
            .collect();
        Self{ angles }
    }

    pub fn n_angles(&self) -> usize {
        self.angles.len()
    }

    pub fn get_angles(&self) -> &[f32] {
        &self.angles
    }
}

impl Default for LidarArray {
    fn default() -> Self {
        LidarArray::new(vec![1.0, 2.0, 5.0, 10.0, 30.0, 45.0, 60.0, 90.0, 120.0])
    }
}

#[derive(Debug, Clone)]
pub enum LidarDistance {
    Specific(f32),
    Far
}


impl Ord for LidarDistance {
    fn cmp(&self, other: &Self) -> Ordering {
        match (self, other) {
            (Self::Far, Self::Far) => panic!("Cannot compare two instances of 'LidarDistance::Far'"),
            (Self::Specific(_), Self::Far) => Ordering::Less,
            (Self::Far, Self::Specific(_)) => Ordering::Greater,
            (Self::Specific(t1), Self::Specific(t2)) => {
                assert!(t1.is_finite());
                assert!(t2.is_finite());
                if t1 < t2 { 
                    Ordering::Less 
                } else if t1 > t2 { 
                    Ordering::Greater
                } else if t1 == t2 {
                    Ordering::Equal
                } else {panic!("Unexpected logic branch");}
            }

        }
    }
}

impl PartialOrd for LidarDistance {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}


impl PartialEq for LidarDistance {
    fn eq(&self, other: &Self) -> bool {
        matches!(self.cmp(other), Ordering::Equal)    
    }
}

impl Eq for LidarDistance {}


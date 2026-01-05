use super::physics::{CarState};
use super::map::{CellMap, LidarArray, Road};


#[derive(Debug)]
pub struct Chunking {
    // Sorted thresholds between the different chunks
    pub distance_thresholds: Vec<f32>, 
}


#[derive(Debug)]
pub struct ChunkingConfig {
    pub lidar: Chunking,
    pub speed: Chunking,
    pub turn: Chunking
}


#[derive(PartialEq, Debug, Hash)]
pub struct ChunkedLidarState {
    pub lidar: Vec<usize>,
    pub speed: usize,
    pub turn: usize
}


impl Chunking {
    fn get_chunk(&self, reading: f32) -> usize {
        if reading < self.distance_thresholds[0] {
            return 0
        }
        let len = self.distance_thresholds.len();
        if reading > self.distance_thresholds[len - 1] {
            return len
        }
        let mut bottom: usize = 0;
        let mut top: usize = self.distance_thresholds.len()-1;

        while top - bottom > 1 {
            let middle = (top + bottom) / 2;

            if reading > self.distance_thresholds[middle] {
                bottom = middle;
            }
            if reading < self.distance_thresholds[middle] {
                top = middle;
            }
        }
        bottom + 1
    }

    pub fn n_chunks(&self) -> usize {
        self.distance_thresholds.len() + 1
    }
}

impl ChunkedLidarState {
    pub fn new(state: &CarState, 
               map: &CellMap, 
               lidar_array: &LidarArray,
               chunking_config: &ChunkingConfig) -> Self {
        let lidar_readings = map.read_lidar(&state, &lidar_array);

        let lidar = lidar_readings.into_iter()
            .map(|reading| chunking_config.lidar.get_chunk(reading))
            .collect();
        Self { 
            lidar,
            speed: chunking_config.speed.get_chunk(state.speed),
            turn: chunking_config.turn.get_chunk(state.steer_delta),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_chunking() {
        let config = Chunking { distance_thresholds: vec![1.0, 2.0, 3.0]};

        assert_eq!(config.get_chunk(0.5), 0);
        assert_eq!(config.get_chunk(1.5), 1);
        assert_eq!(config.get_chunk(2.5), 2);
        assert_eq!(config.get_chunk(3.5), 3);

    }
}


use std::ops::{Index};
use rand::prelude::*;


pub struct Reservoir<T> {
    data: Vec<T>,
    pub capacity: usize,
    n: usize,
    rng: ThreadRng,
}


impl<I, T> Index<I> for Reservoir<T>
where
    Vec<T>: Index<I>,
{
    type Output = <Vec<T> as Index<I>>::Output;

    fn index(&self, index: I) -> &Self::Output {
        &self.data[index]
    }
}


impl<T> Reservoir<T> {
    pub fn new(capacity: usize) -> Self {
        let data = Vec::<T>::with_capacity(capacity);
        Self { data, capacity, n: 0, rng: rand::rng()}
    }

    pub fn update(&mut self, item: T) {

        // If we have not filled the reservoir, we always add the item
        if self.n < self.capacity {
            self.data.push(item);
            self.n += 1;
            return
        } 

        // If the reservoir is filled, we probabilistically replace an item
        let r = self.rng.random::<f32>();
        let p = 1.0 / ( self.n as f32 + 1.0 );
        self.n += 1;

        if r < p {
            // Select random item to replace
            let idx = (0 .. self.data.len()).choose(&mut self.rng)
                .expect("at least one element in buffer; should find index");
            self.data[idx] = item;
        }

    }

    pub fn len(&self) -> usize {
        self.data.len()
    }

    pub fn sample(&mut self) -> Option<&T> {
        (0 .. self.data.len())
            .choose(&mut self.rng)
            .map(|idx| &self.data[idx])
    }

    pub fn sample_batch(&mut self, size: usize) -> Vec<&T> {
        let indices = (0 .. self.data.len())
            .choose_multiple(&mut self.rng, size);

        indices.into_iter().map(|idx| &self.data[idx]).collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_add_before_capacity() {
        let mut reservoir = Reservoir::<f32>::new(5);
        assert_eq!(reservoir.len(), 0);
        reservoir.update(0.0);
        assert_eq!(reservoir.len(), 1);
        reservoir.update(1.0);
        assert_eq!(reservoir.len(), 2);
        reservoir.update(2.0);
        assert_eq!(reservoir.len(), 3);
        reservoir.update(3.0);
        assert_eq!(reservoir.len(), 4);
        reservoir.update(4.0);

        assert_eq!(reservoir[0 .. 5], [0.0, 1.0, 2.0, 3.0, 4.0]);
    }

    #[test]
    fn test_add_after_capacity() {
        let sum: f32 = (0 .. 8000)
            .map(|try_idx| {
                let mut reservoir = Reservoir::<f32>::new(2);

                for i in 0 .. 4 {
                    reservoir.update(i as f32);
                }
                assert_eq!(reservoir.len(), 2);

                let outcome = match (reservoir[0] == 3.0 || reservoir[1] == 3.0) {
                    true => 1.0_f32,
                    false => 0.0_f32
                };
                outcome
            })
            .sum();

        // Check that the value 2.0 was added about 1/3 of te time
        assert!(sum > 1925.0);
        assert!(sum < 2075.0);
    }
}


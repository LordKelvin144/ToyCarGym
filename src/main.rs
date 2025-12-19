mod env;
mod walk;

use walk::{Walk, RandomWalk, Square, Move};
use env::{DeterministicEnv, RandomEnv};

fn main() {
    // let walk = Walk{start: Square(5,5), lower_right: Square(10,10)};
    let walk = RandomWalk::new(0.75);


    let initial_state = Square(0,0);
    let mut state = initial_state.clone();
    println!("initial_state={}", &initial_state);

    for _ in 0..12 {
        state = walk.sample_next_state(&state, &Move::Down);
        println!("state={}", &state);
    }
}

use car_rl::env::{RandomEnv, Env};
use car_rl::walk::{RandomWalk, Move, Square};

use car_rl::tabular_rl::{QTable, Transition};

fn main() {
    let walk = RandomWalk::new(0.8);
    let mut qtable = QTable::<Square,Move>::new();
    let mut epsilon = 1.0;
    let alpha0 = 0.4;
    let mut return_ = 0.0;

    for episode in 1 .. 1000000 {
        let mut state = walk.initial_state();
        let mut this_return = 0.0;

        let alpha = alpha0 / (1.0 + 0.0005*episode as f32);

        for _t in 1 ..= 6 {
            let actions = walk.possible_actions(&state);

            let action = qtable.epsilon_greedy_action(&state, &actions, epsilon);
            // println!("state={:?}, action={:?}, epsilon={}", &state, &action, epsilon);

            // Do the dynamics
            let next_state = walk.sample_next_state(&state, &action);

            this_return += walk.reward(&state, &action, &next_state);
            let transition = Transition { state, action: action, next_state: next_state.clone() };

            qtable.q_learning_step(&walk, transition, |s| s.clone(), alpha, 0.7);

            state = next_state;
            epsilon *= 0.9999999;
        }

        return_ = 0.9999*return_ + 0.0001*this_return;

        if episode % 10000 == 0 {
            println!("Final state: {:?}; return={}; epsilon={}; alpha={}", &state, return_, epsilon, alpha);
            for action in walk.possible_actions(&state).into_iter() {
                println!("Q(origin,{:?})={}", action, qtable.get((&Square(0,0), &action)));
            }
        }
    }


}

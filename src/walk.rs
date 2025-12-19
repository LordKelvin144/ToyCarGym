use crate::env::{Env, DeterministicEnv, RandomEnv};
use std::fmt;


#[derive(Debug,Clone)]
pub enum Move {
    Up,
    Down,
    Left,
    Right
}

impl fmt::Display for Move {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match self {
            Move::Up => write!(f, "up"),
            Move::Down => write!(f, "down"),
            Move::Left => write!(f, "left"),
            Move::Right => write!(f, "right")
        }
    }
}

#[derive(Debug,Clone)]
pub struct Square(pub i32, pub i32);

impl fmt::Display for Square {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "Square({},{})", self.0, self.1)
    }
}

pub struct Walk {
    pub lower_right: Square,
    pub start: Square
}


impl Env<Square, Move> for Walk {
    fn possible_actions(&self, _state: &Square) -> Vec<Move> {
        vec![Move::Up, Move::Down, Move::Left, Move::Right]
    }

    fn reward(&self, _state: &Square, _action: &Move, next_state: &Square) -> f32 {
        match next_state {
            Square(5,0) => 1.0,
            _ => 0.0,
        }
    }

    fn initial_state(&self) -> Square {
        self.start.clone()
    }
}

impl DeterministicEnv<Square, Move> for Walk {
    fn next_state(&self, state: &Square, action: &Move) -> Square {
        let proposed_square = match action {
            Move::Up => Square(state.0 - 1, state.1),
            Move::Down => Square(state.0 + 1, state.1),
            Move::Left => Square(state.0, state.1 -1),
            Move::Right => Square(state.0, state.1 + 1),
        };

        // If the proposed state is valid, return it, otherwise stay at current square
        if proposed_square.0 <= self.lower_right.0 && proposed_square.1 <= self.lower_right.1 
                && proposed_square.0 >= 0 && proposed_square.1 >= 0 {
            proposed_square
        } else {
            state.clone()
        }
    }
}

pub struct RandomWalk {
    pub start: Square,
    p: f32
}

impl RandomWalk {
    pub fn new(success_probability: f32) -> Self {
        if success_probability < 0.0 || success_probability > 1.0 {
            panic!("Success probability must be between 0 and 1");
        };
        Self { start: Square(0, 0),  p: success_probability }
    }
}

impl Env<Square, Move> for RandomWalk {
    fn possible_actions(&self, _state: &Square) -> Vec<Move> {
        vec![Move::Up, Move::Down, Move::Left, Move::Right]
    }

    fn reward(&self, _state: &Square, _action: &Move, next_state: &Square) -> f32 {
        match next_state {
            Square(5,0) => 1.0,
            _ => 0.0,
        }
    }

    fn initial_state(&self) -> Square {
        self.start.clone()
    }
}

impl RandomEnv<Square, Move> for RandomWalk {
    fn transition(&self, state: &Square, action: &Move) -> Vec<(Square, f32)> {
        let all_neighbors: [Square; 4] = [
            Square(state.0 - 1, state.1),
            Square(state.0 + 1, state.1),
            Square(state.0, state.1-1),
            Square(state.0, state.1+1)
        ];

        // Extract the relevant neighbor square
        let desired_idx = match action {
            Move::Up => 0,
            Move::Down => 1,
            Move::Left => 2,
            Move::Right => 3
        };

        // Extract the desired square, and the non-desired squares
        // let desired_square = all_neighbors[desired_idx];
        let remaining_squares: Vec<&Square> = all_neighbors.iter()
            .enumerate()
            .filter_map(|(i,square)| if i != desired_idx { Some(square) } else {None})
            .collect();
        let n_remaining_squares = remaining_squares.len() as f32;

        // Distribute probabilities;
        // the desired square recieves probability p, the remaining probability is distributed
        // among the remaining squares.
        all_neighbors.iter()
            .enumerate()
            .map(|(idx, square)| {
                if idx == desired_idx {
                    (square.clone(), self.p)
                } else {
                    (square.clone(), (1.0 - self.p) / n_remaining_squares)
                }
            })
            .collect()
    }
}

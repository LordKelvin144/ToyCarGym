use crate::env::Env;
use std::hash::Hash;
use std::collections::HashMap;

use std::cmp::{Eq, Ordering};
use std::clone::Clone;

use rand::prelude::{IndexedRandom, Rng};


pub struct QTable<S: Hash+Eq, A: Hash+Eq+Clone> {
    // A lookup holding Q(s,a)
    lookup: HashMap<S, HashMap<A, f32>>
}

#[derive(Debug)]
pub struct Transition<S,A> {
    pub state: S,
    pub action: A,
    pub next_state: S
}


impl<So,A> QTable<So,A>
where 
    So: Hash+Eq,
    A: Hash+Eq+Clone
{
    pub fn new() -> Self {
        Self { lookup: HashMap::<So,HashMap<A, f32>>::new() }
    }

    pub fn get(&self, (s,a): (&So,&A)) -> f32 {
        let a_table = match self.lookup.get(s) {
            Some(table) => table,
            None => {return 0.0;}
        };
        match a_table.get(a) {
            Some(&value) => value,
            None => 0.0
        }
        
    }

    pub fn set(&mut self, (s,a): (So,A), value: f32) {
        let a_table = self.lookup.entry(s)
            .or_insert(HashMap::<A, f32>::new());
        a_table.insert(a, value);
    }

    /// A single step of the Q learning algorithm
    pub fn q_learning_step<Se, F: Fn(&Se) -> So>
        (&mut self,
         env: &impl Env<Se,A>,
         transition: Transition<Se,A>,
         observe_projection: F,
         lr: f32,
         gamma: f32)
    {
        let Transition { state: true_state, action, next_state: true_next_state } = transition;
        let state = observe_projection(&true_state);
        let next_state = observe_projection(&true_next_state);

        // Compute the Q learning update
        let old_q = self.get((&state, &action));
        let new_q = (1.0-lr) * old_q + lr * (
            env.reward(&true_state, &action, &true_next_state)
            + gamma * env.possible_actions(&true_next_state)
                .into_iter()
                .map(|next_action| self.get((&next_state, &next_action)))
                .reduce(f32::max)
                .expect("at least one action to be available")
        );

        // Set the value
        self.set((state, action), new_q);
    }

    pub fn greedy_action(&self, state: &So, actions: &[A]) -> A {
        actions.into_iter()
            .map(|action| { (self.get((state, action)), action) })
            .reduce(|(q, action), (other_q, other_action)| {
                match q.partial_cmp(&other_q) {
                    Some(Ordering::Less) => (other_q, other_action),
                    Some(Ordering::Greater) => (q, action),
                    Some(Ordering::Equal) => (q, action),
                    None => panic!("q values should be finite")
                }
            })
            .expect("at least one action to exist")
            .1
            .clone()
    }

    pub fn epsilon_greedy_action(&self, state: &So, actions: &[A], epsilon: f32) -> A {
        let mut rng = rand::rng();

        if rng.random::<f32>() < epsilon {
            actions.choose(&mut rng)
                .expect("at least one action to exist")
                .clone()
        } else {
            self.greedy_action(state, actions)
        }

    }
}



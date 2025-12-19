use rand::Rng;

/// An environment
pub trait Env<S,A> {
    fn possible_actions(&self, state: &S) -> Vec<A>;
    fn reward(&self, state: &S, action: &A, next_state: &S) -> f32;
    fn initial_state(&self) -> S;
}

/// An environment where the transition dynamics are deterministic.
/// Requires a method for computing the next state, and a method for the reward.
pub trait DeterministicEnv<S,A>: Env<S,A> {
    fn next_state(&self, state: &S, action: &A) -> S;
}

/// An environment where the transition dynamics are random.
/// Provides a method for sampling the next state.
pub trait RandomEnv<S,A>: Env<S,A> {
    /// Computes the possible next states and their respective probabilities
    fn transition(&self, state: &S, action: &A) -> Vec<(S, f32)>;

    /// Sample the next state
    fn sample_next_state(&self, state: &S, action: &A) -> S {
        let mut rng = rand::rng();
        let r: f32 = rng.random();
        let transitions = self.transition(state, action);

        // Sample a transition using CDF sampling
        let mut cumsum: f32 = 0.0;
        let mut sampled = None;
        for (next_state, p) in transitions {
            cumsum += p;
            if cumsum > r {
                sampled = Some(next_state);
                break;
            }
        };
        sampled.expect("Sampling procedure to find state.")
    }
}

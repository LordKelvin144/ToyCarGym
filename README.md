# ToyCarGym - A simple car physics simulator
This project is a lightweight 2D car physics simulator written in Rust, with a Gym-like Python API exposed via PyO3 and maturin. The goal was to develop a toy environment that could be used for personal reinforcement learning experimentation. The physics is deterministic, with seedable random initialization, and the dynamics are stable even with larger time steps.

Additionally, to facilitate debugging, a human-facing game client is available in the `car_game` crate. This crate and the corresponding dependencies are entirely decoupled from the simulator and are not built if using only the Python bindings.

--- 

## Design goals

- **Simple physics**: Uses the bicycle model to model the car, giving stable analytical dynamics while being detailed enough to be challenging for RL
- **Deterministic simulation**: The dynamics are deterministic, and the random initialization is entirely seedable
- **Fast**: Rust handles simulation and LIDAR observation logic, while exposing Python bindings for convenience in downstream tasks
- **Modular build**: Python bindings do not depend on graphics or windowing libraries

---

## Project structure
```
ToyCarGym
├── car_game/  # Binary crate for running an interactive game
├── Cargo.toml 
├── car_sim/src  # The simulator core
│   ├── gym.rs  # Underlying Rust implementation for Gym API
│   ├── lib.rs
│   ├── lidar.rs  # Lidar utilities
│   ├── map/  # Collision/intersection checking for spline-based and grid-based maps
│   └── physics.rs  # Implementation of physics/input dynamics
├── graphics_utils/  # Graphics utilities (e.g. spline graphics)
├── gym_car/  # Python bindings
├── math_utils/  # Vector arithmetic and spline computations 
└── README.md
```
Using this structure, the `car_game` and `graphics_utils` crates are completely separate from the core engine, and graphics-related code is only built when building these crates specifically.

## Installation (Python bindings)
Clone the repo and create a virtual environment:
```bash
python3 -m venv venv
source venv/bin/activate
```
Install the maturin build system for exporting Python bindings, and numpy if not already present
```bash
pip install maturin numpy
```
Install the `gym_car` package to the environment:
```bash
cd gym_car
maturin develop
```
For an optimized build:
```bash
maturin develop --release
```

## Python API example
```python
from gym_car import RacingEnv

# Create environment
env = RacingEnv(dt=0.01, seed=0)

# Reset environment
env.reset(seed=1)

# Take a step (of size dt) in the environment while accelerating
# Actions are [left, right, accelerate, brake, coast]
reward, done = env.step(action=2)

# Observe the environment (a vector containing lidar readings, steering angle and speed)
observation = env.observe()
```
Note that the API is similar to the OpenAI Gym API, but the state observation is implemented as a separate call. This is because the transition dynamics and state observation are entirely decoupled in the underlying physics, and since `env.observe()` is generally much more computationally expensive than `env.step()`, I opted to explicitly separate them.

## Running the Rust game client
The crate `car_game` provides a graphical frontend built with `macroquad`.

Run it with:
```bash
cargo run -p car_game --release
```

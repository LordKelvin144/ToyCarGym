import numpy as np
import numpy.typing as npt

from typing import Tuple, List, Optional

class RacingEnv:
    def __init__(
            self,
            dt: Optional[float] = None,
            crash_reward: Optional[float] = None,
            travel_coeff: Optional[float] = None,
            center_coeff: Optional[float] = None,
            center_integral_coeff: Optional[float] = None,
            observe_delta: Optional[bool] = True,
            observe_speed: Optional[bool] = True,
        ):
        """
        Create a new racing environment.

        Creates a racing environment with user-provided settings.

        Parameters
        ----------
        dt : float, optional
            The time step used in simulation.
        crash_reward : float, optional
            The reward given when crashing.
        travel_coeff : float, optional
            The positive reward given per meter travelled along the track.
        center_coeff : float, optional
            The strength of the potential-based penalty for deviating from the center-line. Given as positive [penalty/meter^2].
        center_coeff : float, optional
            The strength of the time-integral-based penalty for deviating from the center-line. Given as positive [penalty/(second*meter^2)].
        observe_delta : bool,
            Whether to include the wheel steering angle as part of the state observation. Default is True.
        observe_speed : bool
            Whether to include the speed as part of the state observation. Default is True.
        """

    def step(self, action: int) -> Tuple[float, bool]:
        """
        Perform a single MDP update step and return transition information.

        Mutates the environment state and returns reward and truncation information.

        Parameters
        ----------
        action : int
            Index of the action to perform in the environment.

        Returns
        -------
        reward : float
            The collected reward for this transition.
        done : bool
            Whether the transition caused the episode to terminate.
        """

    def reset(self):
        """
        Reset the environment.
        """

    def observe(self) -> npt.NDArray[np.float32]:
        """
        Observe the current state of the environment.

        Returns an array representation of the observation of the current state.

        Returns
        -------
        observation : ndarray
            An array of shape (observation_dim,) holding the observations
        """
    @property
    def dt(self) -> float:
        """
        The simulation time step.
        """

    @property
    def t(self) -> float:
        """
        The current time.
        """

    @property
    def i(self) -> int:
        """
        The current MDP iteration.
        """

    @property
    def observation_dim(self) -> int
        """
        The number of features in the observation space.
        """

    def graphics_state(self) -> 'CarGraphicsExport':
        """
        Returns an extended representation of the environment state useful for rendering.

        Returns
        -------
        export : CarGraphicsExport
            A struct holding information about relevant coordinates of points in the simulator environment.
        """

    def export_road(self, n_segments: int) -> 'SplineRoadExport':
        """
        Returns a representation of the coordinates of the road segments useful for rendering.

        Parameters
        ----------
        n_segments : int
            The number of segments to split the road edges into, i.e. the granularity of the exported coordinates.

        Returns
        -------
        export : SplineRoadExport
            A struct holding the static coordinates of the road
        """




class SplineRoadExport:
    @property
    def left_x(self) -> List[float]: ...
    @property
    def left_y(self) -> List[float]: ...
    @property
    def right_x(self) -> List[float]: ...
    @property
    def right_y(self) -> List[float]: ...


class CarGraphicsExport:
    @property
    def car_x(self) -> Tuple[float, float, float, float]: ...
    @property
    def car_y(self) -> Tuple[float, float, float, float]: ...
    @property
    def lidar_center(self) -> Tuple[float, float]: ...
    @property
    def lidar_x(self) -> List[float]: ...
    @property
    def lidar_y(self) ->List[float]: ...


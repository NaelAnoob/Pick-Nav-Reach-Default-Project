from typing import TypeAlias, TypedDict

import numpy as np
import pybullet as p
from numpy.typing import NDArray

from pick_nav_reach_env import PickNavReachEnv


FloatArray: TypeAlias = NDArray[np.float32 | np.float64]


class Observation(TypedDict):
    """Environment observations; array shapes are documented below."""

    qpos: FloatArray            # (15,) joint positions
    qvel: FloatArray            # (15,) joint velocities
    object_pos: FloatArray      # (3,) world position
    object_xyzw: FloatArray     # (4,) orientation quaternion
    goal_pos: FloatArray        # (3,) world position
    object_pc: FloatArray       # (1024, 3) world-space surface points
    object_normals: FloatArray  # (1024, 3) world-space surface normals
    cube_positions: FloatArray  # (N, 2) maze obstacle positions


class StepInfo(TypedDict):
    """Evaluation values returned alongside the next observation."""

    dist_to_goal: np.float64
    success: np.bool_


class HoldController:
    def __init__(self, initial_obs: Observation) -> None:
        self.target_qpos: FloatArray  # (15,); annotation only, no value yet
        # TODO: Store an independent copy of the initial joint positions.
        raise NotImplementedError("Store the initial joint targets here.")

    def get_action(self, obs: Observation) -> FloatArray:
        """Return (15,) absolute joint targets, not movement increments."""
        # TODO: Return a copy of the stored joint targets.
        raise NotImplementedError("Return the stored joint targets here.")


def main() -> None:
    env: PickNavReachEnv = PickNavReachEnv(seed=42, object_idx=5)

    try:
        obs: Observation = env.obs
        controller: HoldController = HoldController(obs)

        for step in range(100):
            action: FloatArray
            info: StepInfo

            # TODO: Ask the controller for an action.

            # TODO: Check that its shape is (env.action_size,)
            #       and that all values are finite.

            # TODO: Apply the action and receive the new obs and info.

            # TODO: Every 20 steps, print the step number,
            #       object position, and distance to the goal.
            pass
    finally:
        p.disconnect(env.pb_physics_client)


if __name__ == "__main__":
    main()

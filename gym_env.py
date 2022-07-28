import gym
from gym import spaces
import pybullet
from gym.core import ObsType, ActType
from pybullet import *
import numpy as np
from scene import create_scene
from scene_functions import *
from typing import *


class Kicker(gym.Env):
    metadata = {'render_modes': ['human', "rgb_array", "single_rgb_array"],
                "render_fps": 4}

    def __init__(self,
                 bullet_connection_type: int = DIRECT,
                 render_mode: str = 'human',
                 player: 1 or 2 = 1,
                 ):
        self.render_mode = render_mode if render_mode in self.metadata[
            'render_modes'] else 'human'

        self.observation_space = spaces.Dict({
            "ball": spaces.Box(-20, 20, (3,)),
            "player1_arms": spaces.Tuple((
                spaces.Box(np.array([-np.pi, -3]),
                           np.array([np.pi, 3])),
                spaces.Box(np.array([-np.pi, -3]),
                           np.array([np.pi, 3]))
            )),
            "player2_arms": spaces.Tuple((
                spaces.Box(np.array([-np.pi, -3]),
                           np.array([np.pi, 3])),
                spaces.Box(np.array([-np.pi, -3]),
                           np.array([np.pi, 3]))
            )),
        })

        self.action_space = spaces.Tuple((
            spaces.Dict({
                "rotator": spaces.Discrete(3, start=-1),
                "slider": spaces.Box(-3, 3),
            }),
            spaces.Dict({
                "rotator": spaces.Discrete(3, start=-1),
                "slider": spaces.Box(-3, 3),
            }),
        ))

        self.pb_connection = connect(bullet_connection_type)
        self.pb_objects, self.pb_zero_state = create_scene(self.pb_connection)

    def _get_obs(self):
        # rotator_id, slider_id = 1, 2
        return {
            "ball": getBasePositionAndOrientation(self.pb_objects['ball'],
                                                  physicsClientId=self.pb_connection)[
                0],
            # TODO: getJoinState -> getJointStates
            "player1_arms": (
                (
                    getJointState(self.pb_objects["player1_arm1"], 1)[0],
                    getJointState(self.pb_objects["player1_arm1"], 2)[0],
                ),
                (
                    getJointState(self.pb_objects["player1_arm2"], 1)[0],
                    getJointState(self.pb_objects["player1_arm2"], 2)[0],
                ),
            ),
            "player2_arms": (
                (
                    getJointState(self.pb_objects["player2_arm1"], 1)[0],
                    getJointState(self.pb_objects["player2_arm1"], 2)[0],
                ),
                (
                    getJointState(self.pb_objects["player2_arm2"], 1)[0],
                    getJointState(self.pb_objects["player2_arm2"], 2)[0],
                ),
            )
        }

    def reset(
            self,
            *,
            seed: Optional[int] = None,
            return_info: bool = False,
            options: Optional[dict] = None,
    ) -> Union[ObsType, Tuple[ObsType, dict]]:
        super().reset(seed=seed)
        reset(self.pb_zero_state,
              (
                  self.pb_objects["player2_arm2"],
                  self.pb_objects["player2_arm1"],
                  self.pb_objects["player1_arm1"],
                  self.pb_objects["player1_arm2"],
              ),
              self.pb_connection)
        return self._get_obs()

    def step(
            self, action: ActType
    ) -> Union[
        Tuple[ObsType, float, bool, bool, dict], Tuple[
            ObsType, float, bool, dict]
    ]:
        arm1_rotator_action = action[0]["rotator"]
        arm1_slider_pos = action[0]["slider"]
        arm2_rotator_action = action[1]["rotator"]
        arm2_slider_pos = action[1]["slider"]

        setJointMotorControl2(
            self.pb_objects["player1_arm1"],
            2,
            POSITION_CONTROL,
            targetPosition=arm1_slider_pos,
        )
        setJointMotorControl2(
            self.pb_objects["player1_arm1"],
            1,
            TORQUE_CONTROL,
            force=2000 * arm1_rotator_action,

        )

        setJointMotorControl2(
            self.pb_objects["player1_arm2"],
            2,
            POSITION_CONTROL,
            targetPosition=arm2_slider_pos,
        )
        setJointMotorControl2(
            self.pb_objects["player1_arm2"],
            1,
            TORQUE_CONTROL,
            force=2000 * arm2_rotator_action,

        )
        done = is_win(self.pb_objects["ball"], self.pb_connection)
        reward = reward_f(self.pb_objects["ball"], self.pb_connection)
        obs = self._get_obs()

        stepSimulation(self.pb_connection)

        return obs, reward, done, {}

    def close(self):
        disconnect(self.pb_connection)

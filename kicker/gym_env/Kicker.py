import gym
from gym import spaces
from gym.core import ObsType, ActType, RenderFrame
import numpy as np
from ..scene.kicker import create_scene
from ..reset_functions import camera_reset, scene_reset
from ..is_done_functions import is_done
from ..reward_functions import advanced_reward_function
import pybullet as pb
from pybullet import *
from typing import Tuple, Union, Optional, List, Callable, Dict
from gym.utils.renderer import Renderer
from ..ai import simple_bot

# types for typing
physicsClientId = int


class KickerEnv(gym.Env):
    metadata = {'render.modes': ['human', 'rgba_array']}

    def __init__(self,
                 bullet_connection_type: int = pb.GUI,
                 render_mode: str = 'human',
                 render_resolution: Tuple[int, int] = (1024, 800),
                 ai_function: Optional[Callable[
                     [Dict[str, int], physicsClientId], Callable]] = simple_bot,
                 reward_function: Callable[
                     [Tuple[float, float, float], physicsClientId], float] = advanced_reward_function,
                 player: 1 or 2 = 1,  # unused for now
                 max_steps: Optional[int] = None
                 ):
        """

        :param bullet_connection_type: connection type from pybullet(Direct, GUI?) <br>
        :param render_mode: use "rgba_array" for capturing images every step. You can get array of images using the .render() method
        :param render_resolution: tuple of width and height of window or image(in "rgba_array" mode)
        :param ai_function: <b> can be None</b> function that accepts dictionary of objects (and physicsClientId) and return function that control player on call
        :param reward_function: Function that accepts: (Arm1 unique id, Arm2 unique id, ball unique id, physicsClientId)
        :param player: Which player will be controlled by env (Unused!)
        """
        self.camera_width, self.camera_height = render_resolution

        self.render_mode = render_mode if render_mode in self.metadata[
            'render.modes'] else 'human'

        self.max_steps = max_steps

        self.observation_space = spaces.Dict({
            "ball": spaces.Box(-20, 20, (3,)),
            "player1_arms": spaces.Tuple((
                # arm 1
                spaces.Box(np.array([-np.pi, -3]),
                           np.array([np.pi, 3])),
                # arm 2
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

        # https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit#heading=h.jxof6bt5vhut
        self.action_space = spaces.Tuple((
            spaces.Dict({
                # "rotator": spaces.Discrete(3, start=-1),  # direction of force application
                "rotator": spaces.Dict({
                    "velocity": spaces.Box(-20, 20),  # velocity
                    "k": spaces.Box(2, 2),  # velocity gain(?)
                }),
                # "slider": spaces.Box(-3, 3),  # "middle player" position
                "slider": spaces.Dict({
                    "velocity": spaces.Box(-20, 20),  # velocity
                    "k": spaces.Box(2, 2),  # velocity gain(?)
                }),
            }),
            spaces.Dict({
                "rotator": spaces.Dict({
                    "velocity": spaces.Box(-20, 20),  # velocity
                    "k": spaces.Box(2, 2),  # velocity gain(?)
                }),
                "slider": spaces.Dict({
                    "velocity": spaces.Box(-20, 20),  # velocity
                    "k": spaces.Box(2, 2),  # velocity gain(?)
                }),
            }),
        ))

        self.pb_connection = pb.connect(
            bullet_connection_type,
            options=f"--width={render_resolution[0]} --height={render_resolution[1]}"
        )
        configureDebugVisualizer(COV_ENABLE_KEYBOARD_SHORTCUTS, 0,
                                 physicsClientId=self.pb_connection)
        configureDebugVisualizer(COV_ENABLE_GUI, 0,
                                 physicsClientId=self.pb_connection)

        # matrix for screenshots
        self.viewMatrix = computeViewMatrixFromYawPitchRoll(
            cameraTargetPosition=(0, 0, 0),
            distance=20,
            yaw=90.,
            pitch=-89.999999,
            roll=0.,
            upAxisIndex=2,
            physicsClientId=self.pb_connection
        )

        # for screenshots
        self.projectionMatrix = computeProjectionMatrixFOV(
            90,
            render_resolution[0] / render_resolution[1],
            0,
            20
        )

        camera_reset(self.pb_connection)

        self.pb_objects, self.pb_zero_state = create_scene(self.pb_connection)

        self.renderer = None
        # if render_mode != "human":
        self.renderer = Renderer(self.render_mode, self._render_frame)

        assert reward_function is not None, "No reward function passed"
        self.reward_function = reward_function

        self.ai_bot = ai_function(
            self.pb_objects,
            self.pb_connection
        ) if ai_function else None

    def _render_frame(self, mode: str):
        stepSimulation(self.pb_connection)
        if mode == "rgba_array":
            return getCameraImage(
                self.camera_width,
                self.camera_height,
                viewMatrix=self.viewMatrix,
                projectionMatrix=self.projectionMatrix,
                physicsClientId=self.pb_connection,
            )[2]

    def render(self, mode="human") -> Optional[
        Union[RenderFrame, List[RenderFrame]]]:
        return self.renderer.get_renders()

    def _get_obs(self):
        # rotator_id, slider_id = 1, 2
        p1a1 = getJointStates(self.pb_objects["player1_arm1"], (1, 2))
        p1a2 = getJointStates(self.pb_objects["player1_arm2"], (1, 2))
        p2a1 = getJointStates(self.pb_objects["player2_arm1"], (1, 2))
        p2a2 = getJointStates(self.pb_objects["player2_arm2"], (1, 2))
        return {
            "ball": getBasePositionAndOrientation(
                self.pb_objects['ball'],
                physicsClientId=self.pb_connection
            )[0],
            "player1_arms": (
                (
                    p1a1[0][0],
                    p1a1[1][0],
                ),
                (
                    p1a2[0][0],
                    p1a2[1][0],
                ),
            ),
            "player2_arms": (
                (
                    p2a1[0][0],
                    p2a1[1][0],
                ),
                (
                    p2a2[0][0],
                    p2a2[1][0],
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
        scene_reset(self.pb_zero_state,
                    (
                        self.pb_objects["player2_arm2"],
                        self.pb_objects["player2_arm1"],
                        self.pb_objects["player1_arm1"],
                        self.pb_objects["player1_arm2"],
                    ),
                    self.pb_connection)
        self.renderer.reset()
        self.renderer.render_step()
        self.step_cnt = 0
        return self._get_obs()

    def step(
            self, action: ActType
    ) -> Union[
        Tuple[ObsType, float, bool, bool, dict], Tuple[
            ObsType, float, bool, dict]
    ]:
        self.step_cnt += 1
        rotator_id, slider_id = 1, 2
        arm1_rotator_velocity = action[0]["rotator"]["velocity"]
        arm1_rotator_k = action[0]["rotator"]["k"]
        arm1_slider_velocity = action[0]["slider"]["velocity"]
        arm1_slider_k = action[0]["slider"]["k"]
        arm2_rotator_velocity = action[1]["rotator"]["velocity"]
        arm2_rotator_k = action[1]["rotator"]["k"]
        arm2_slider_velocity = action[1]["slider"]["velocity"]
        arm2_slider_k = action[1]["slider"]["k"]

        # setting new state

        setJointMotorControlArray(
            self.pb_objects["player1_arm1"],
            (rotator_id, slider_id),
            pb.VELOCITY_CONTROL,
            targetVelocities=(arm1_rotator_velocity, arm1_slider_velocity),
            velocityGains=(arm1_rotator_k, arm1_slider_k),
            physicsClientId=self.pb_connection
        )
        setJointMotorControlArray(
            self.pb_objects["player1_arm2"],
            (rotator_id, slider_id),
            pb.VELOCITY_CONTROL,
            targetVelocities=(arm2_rotator_velocity, arm2_slider_velocity),
            velocityGains=(arm2_rotator_k, arm2_slider_k),
            physicsClientId=self.pb_connection
        )
        ball_cds = getBasePositionAndOrientation(self.pb_objects["ball"],
                                                 self.pb_connection)[0]
        done = bool(is_done(ball_cds))
        if self.max_steps is not None and self.step_cnt > self.max_steps:
            done = True

        reward = self.reward_function(ball_cds, self.pb_connection)
        obs = self._get_obs()

        if self.ai_bot:
            self.ai_bot(ball_cds)

        # stepSimulation(self.pb_connection)
        self.renderer.render_step()

        if self.renderer:
            self.renderer.render_step()

        return obs, reward, done, {}

    def close(self):
        disconnect(self.pb_connection)

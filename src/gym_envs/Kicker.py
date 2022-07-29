import gym
from gym import spaces
from gym.core import ObsType, ActType, RenderFrame
import numpy as np
from src.scene.kicker import create_scene
from src.utils.scene_functions import *
from typing import Tuple, Union, Optional, List
from gym.utils.renderer import Renderer
from src.ai.firstSimple import create_bot


class KickerEnv(gym.Env):
    metadata = {'render_modes': ['human', 'rgba_array']}

    def __init__(self,
                 bullet_connection_type: int = pb.GUI,
                 render_mode: str = 'human',
                 render_resolution: Tuple[int, int] = (1024, 800),
                 enable_ai: bool = True,
                 player: 1 or 2 = 1,  # unused for now
                 ):

        self.camera_width, self.camera_height = render_resolution

        self.render_mode = render_mode if render_mode in self.metadata[
            'render_modes'] else 'human'

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

        # TODO: edit rotator:
        # https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit#heading=h.jxof6bt5vhut
        self.action_space = spaces.Tuple((
            spaces.Dict({
                # "rotator": spaces.Discrete(3, start=-1),  # direction of force application
                "rotator": spaces.Dict({
                    "velocity": spaces.Box(-30, 30),  # velocity
                    "k": spaces.Box(-30, 30),  # velocity gain(?)
                }),
                # "slider": spaces.Box(-3, 3),  # "middle player" position
                "slider": spaces.Dict({
                    "velocity": spaces.Box(-30, 30),  # velocity
                    "k": spaces.Box(-30, 30),  # velocity gain(?)
                }),
            }),
            spaces.Dict({
                "rotator": spaces.Dict({
                    "velocity": spaces.Box(-30, 30),  # velocity
                    "k": spaces.Box(-30, 30),  # velocity gain(?)
                }),
                "slider": spaces.Dict({
                    "velocity": spaces.Box(-30, 30),  # velocity
                    "k": spaces.Box(-30, 30),  # velocity gain(?)
                }),
            }),
        ))

        self.pb_connection = pb.connect(bullet_connection_type, options=f"--width={render_resolution[0]} --height={render_resolution[1]}")
        configureDebugVisualizer(COV_ENABLE_KEYBOARD_SHORTCUTS, 0, physicsClientId=self.pb_connection)
        configureDebugVisualizer(COV_ENABLE_GUI, 0, physicsClientId=self.pb_connection)
        camera_reset(self.pb_connection)
        self.pb_objects, self.pb_zero_state = create_scene(self.pb_connection)

        self.renderer = None
        if render_mode != "human":
           self.renderer = Renderer(self.render_mode, self._render_frame)

        self.ai_bot = create_bot(
            self.pb_objects["player2_arm1"],
            self.pb_objects["player2_arm2"],
            self.pb_objects["ball"],
            self.pb_connection
        ) if enable_ai else None

    def _render_frame(self):
        return getCameraImage(
            self.camera_width,
            self.camera_height,
            physicsClientId=self.pb_connection,
        )[2]

    def render(self, mode="human") -> Optional[Union[RenderFrame, List[RenderFrame]]]:
        return self.renderer.get_renders()

    def _get_obs(self):
        # rotator_id, slider_id = 1, 2
        p1a1 = getJointStates(self.pb_objects["player1_arm1"], (1, 2))
        p1a2 = getJointStates(self.pb_objects["player1_arm1"], (1, 2))
        p2a1 = getJointStates(self.pb_objects["player1_arm1"], (1, 2))
        p2a2 = getJointStates(self.pb_objects["player1_arm1"], (1, 2))
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
        return self._get_obs()

    def step(
            self, action: ActType
    ) -> Union[
        Tuple[ObsType, float, bool, bool, dict], Tuple[
            ObsType, float, bool, dict]
    ]:
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

        done = bool(is_done(self.pb_objects["ball"], self.pb_connection))
        reward = simple_reward(self.pb_objects["ball"], self.pb_connection)
        obs = self._get_obs()

        if self.ai_bot:
            self.ai_bot()

        stepSimulation(self.pb_connection)
        if self.renderer:
            self.renderer.render_step()

        return obs, reward, done, {}

    def close(self):
        disconnect(self.pb_connection)

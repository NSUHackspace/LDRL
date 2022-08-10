import gym
from gym import spaces
import numpy as np
from ..scene.kicker import create_scene
from ..reset_functions import camera_reset, scene_reset
from pybullet import resetDebugVisualizerCamera
from ..is_done_functions import is_done
from ..reward_functions import advanced_reward_function
import pybullet as pb
from pybullet import *
from typing import Tuple, Union, Optional, List, Callable, Dict
from ..ai import simple_bot

# types for typing
physicsClientId = int
ball_coordinates = Tuple[float, float, float]
object_list = Dict[str, int]


class KickerEnv(gym.Env):
    metadata = {'render.modes': ['human', 'rgba_array']}

    def __init__(self,
                 bullet_connection_type: int = pb.GUI,
                 render_mode: str = 'human',
                 render_resolution: Tuple[int, int] = (1024, 800),
                 ai_function: Optional[Callable[
                     [object_list, physicsClientId], Callable[
                         [ball_coordinates], None]]] = simple_bot,
                 reward_function: Callable[
                     [ball_coordinates,
                      physicsClientId], float] = advanced_reward_function,
                 player: 1 or 2 = 1,  # unused for now
                 max_steps: Optional[int] = None,
                 ball_coords: Optional[int] = 2
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

        self.ball_coords = ball_coords
        if ball_coords == 2:
            ball_obs = spaces.Box(-1, 1, (2,))
        elif ball_coords == 3:
            ball_obs = spaces.Box(-20, 20, (3,))
        else:
            raise Exception("2 or 3 ball coords is supported")

        print(ball_obs)

        self.observation_space = spaces.Dict({
            "ball": ball_obs,
            "player1_arms": spaces.Tuple((
                # arm 1
                spaces.Dict({
                    "rotator": spaces.Box(np.array([-np.pi]),
                                          np.array([np.pi])),
                    "slider": spaces.Box(np.array([-3]), np.array([3]))
                }),
                # arm 2
                spaces.Dict({
                    "rotator": spaces.Box(np.array([-np.pi]),
                                          np.array([np.pi])),
                    "slider": spaces.Box(np.array([-3]), np.array([3]))
                }),
            )),
            "player2_arms": spaces.Tuple((
                # arm 1
                spaces.Dict({
                    "rotator": spaces.Box(np.array([-np.pi]),
                                          np.array([np.pi])),
                    "slider": spaces.Box(np.array([-3]), np.array([3]))
                }),
                # arm 2
                spaces.Dict({
                    "rotator": spaces.Box(np.array([-np.pi]),
                                          np.array([np.pi])),
                    "slider": spaces.Box(np.array([-3]), np.array([3]))
                }),
            )),
        })

        # https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit#heading=h.jxof6bt5vhut
        self.action_space = spaces.Tuple((
            spaces.Dict({
                # "rotator": spaces.Discrete(3, start=-1),  # direction of force application
                "rotator": spaces.Dict({
                    "velocity": spaces.Box(-20, 20, (1,)),  # velocity
                    "k": spaces.Box(2, 2, (1,)),  # velocity gain(?)
                }),
                # "slider": spaces.Box(-3, 3),  # "middle player" position
                "slider": spaces.Dict({
                    "velocity": spaces.Box(-20, 20, (1,)),  # velocity
                    "k": spaces.Box(2, 2, (1,)),  # velocity gain(?)
                }),
            }),
            spaces.Dict({
                "rotator": spaces.Dict({
                    "velocity": spaces.Box(-20, 20, (1,)),  # velocity
                    "k": spaces.Box(2, 2, (1,)),  # velocity gain(?)
                }),
                "slider": spaces.Dict({
                    "velocity": spaces.Box(-20, 20, (1,)),  # velocity
                    "k": spaces.Box(2, 2, (1,)),  # velocity gain(?)
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

        cam_camera_target_position=(0, 0, 0)
        cam_distance=20.
        cam_yaw=90.
        cam_pitch=-89.99999
        cam_roll=0.
        cam_up_axis_index=2

        resetDebugVisualizerCamera(
                cameraDistance=cam_distance,
                cameraYaw=cam_yaw,
                cameraPitch=cam_pitch,
                cameraTargetPosition=cam_camera_target_position,
                physicsClientId=self.pb_connection
        )

        self.view_matrix = computeViewMatrixFromYawPitchRoll(
            cameraTargetPosition=cam_camera_target_position,
            distance=cam_distance,
            yaw=cam_yaw,
            pitch=cam_pitch,
            roll=cam_roll,
            upAxisIndex=2,
            physicsClientId=self.pb_connection
        )

        self.projection_matrix = computeProjectionMatrixFOV(
            fov=90,
            aspect=render_resolution[0] / render_resolution[1],
            nearVal=0.1,
            farVal=100
        )

        projection_matrix_np = np.array(self.projection_matrix).reshape(4, 4)
        view_matrix_np = np.array(self.view_matrix).reshape(4, 4)
        self.mvp_matrix = projection_matrix_np.T @ view_matrix_np.T

        self.pb_objects, self.pb_zero_state = create_scene(self.pb_connection)

        assert reward_function is not None, "No reward function passed"
        self.reward_function = reward_function

        self.ai_bot = ai_function(
            self.pb_objects,
            self.pb_connection
        ) if ai_function else None

    def render(self, mode: str):
        if mode == "rgba_array":
            return getCameraImage(
                self.camera_width,
                self.camera_height,
                viewMatrix=self.view_matrix,
                projectionMatrix=self.projection_matrix,
                physicsClientId=self.pb_connection,
            )[2]
        else:
            super().render(mode=mode)

    def _project_coords(self, x, y, z):
        v = np.array((x, y, z, 1))
        pr_v =  self.mvp_matrix @ v.T
        pr_v = pr_v / pr_v[-1]
        return pr_v[:2]

    def _get_obs(self):
        ball_cds = getBasePositionAndOrientation(self.pb_objects["ball"],
                                                 self.pb_connection)[0]

        ball_cds_2d = self._project_coords(*ball_cds)
        # rotator_id, slider_id = 1, 2
        p1a1 = getJointStates(self.pb_objects["player1_arm1"], (1, 2))
        p1a2 = getJointStates(self.pb_objects["player1_arm2"], (1, 2))
        p2a1 = getJointStates(self.pb_objects["player2_arm1"], (1, 2))
        p2a2 = getJointStates(self.pb_objects["player2_arm2"], (1, 2))
        return {
            "ball": ball_cds if self.ball_coords == 3 else ball_cds_2d,
            "player1_arms": (
                ({
                    "rotator": p1a1[0][0],
                    "slider": p1a1[1][0],
                }),
                ({
                    "rotator": p1a2[0][0],
                    "slider": p1a2[1][0],
                }),
            ),
            "player2_arms": (
                ({
                    "rotator": p2a1[0][0],
                    "slider": p2a1[1][0],
                }),
                ({
                    "rotator": p2a2[0][0],
                    "slider": p2a2[1][0],
                }),
            )
        }

    def reset(
            self,
            *,
            seed: Optional[int] = None,
            return_info: bool = False,
            options: Optional[dict] = None,
    ):
        scene_reset(self.pb_zero_state,
                    (
                        self.pb_objects["player2_arm2"],
                        self.pb_objects["player2_arm1"],
                        self.pb_objects["player1_arm1"],
                        self.pb_objects["player1_arm2"],
                    ),
                    self.pb_connection)
        self.step_cnt = 0
        return self._get_obs()

    def step(self, action):
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

        stepSimulation(self.pb_connection)

        ball_cds = getBasePositionAndOrientation(self.pb_objects["ball"],
                                                 self.pb_connection)[0]

        done = bool(is_done(ball_cds))
        if self.max_steps is not None and self.step_cnt > self.max_steps:
            done = True

        reward = self.reward_function(ball_cds, self.pb_connection)
        obs = self._get_obs()

        if self.ai_bot:
            self.ai_bot(ball_cds)

        return obs, reward, done, {}

    def close(self):
        disconnect(self.pb_connection)

#!/usr/bin/env python3

import os
from datetime import datetime
from typing import Callable

import gym
import pybullet as pb
from gym import ActionWrapper
from gym.wrappers import FlattenObservation, FrameStack, TimeLimit
from kicker.ai.rotate_to_target import create_rotate_to_target_bot
from kicker.callbacks import GoalCallback
from kicker.gym_env import KickerEnv
from kicker.reset_functions import camera_reset
from kicker.reward_functions import simple_reward, advanced_reward_function
from kicker.wrappers import FlattenAction
from stable_baselines3 import A2C
from stable_baselines3.common.callbacks import CheckpointCallback
from stable_baselines3.common.logger import configure
from stable_baselines3.common.utils import set_random_seed
from stable_baselines3.common.vec_env import DummyVecEnv, SubprocVecEnv


def make_env(rank: int, seed: int = 0) -> Callable:
    """
    Utility function for multiprocessed env.

    :param env_id: (str) the environment ID
    :param num_env: (int) the number of environment you wish to have in subprocesses
    :param seed: (int) the inital seed for RNG
    :param rank: (int) index of the subprocess
    :return: (Callable)
    """

    def _init() -> gym.Env:
        env = FlattenAction(
            FlattenObservation(
                KickerEnv(
                    bullet_connection_type=pb.DIRECT,
                    reward_function=advanced_reward_function,
                    # ai_function=create_rotate_to_target_bot,
                    ai_function=None,
                    ball_init_lim_x = (-1, 1),
                    ball_init_lim_y = (-1, 1)
                )
            )
        )
        env = TimeLimit(env, max_episode_steps=10000)
        env = FrameStack(env, 2)
        env.seed(seed + rank)
        return env

    set_random_seed(seed)
    return _init


def main():
    # num_cpu = 4  # Number of processes to use
    # # Create the vectorized environment
    # env = SubprocVecEnv([make_env(i) for i in range(num_cpu)])
    env = make_env(0)()

    timestamp = datetime.now().strftime("%Y-%m-%d.%H-%M-%S")

    model_dir = f"model/a2c_{timestamp}"
    if not os.path.exists(model_dir):
        os.makedirs(model_dir)

    logdir = f"logs/a2c_{timestamp}"
    if not os.path.exists(logdir):
        os.makedirs(logdir)

    checkpoint_callback = CheckpointCallback(
        save_freq=500000, save_path=model_dir, name_prefix="a2c"
    )
    goal_callback = GoalCallback()
    new_logger = configure(logdir, ["csv", "tensorboard"])

    model = A2C("MlpPolicy", env, verbose=1)
    model.set_logger(new_logger)
    model.learn(
        total_timesteps=10e6,
        callback=[goal_callback, checkpoint_callback],
    )

    env.close()


if __name__ == "__main__":
    main()

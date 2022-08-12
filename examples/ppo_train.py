#!/usr/bin/env python3

from kicker.gym_env import KickerEnv
import pybullet as pb
from kicker.reset_functions import camera_reset
from stable_baselines3 import PPO
from gym.wrappers import FlattenObservation
from gym.wrappers import FrameStack
from gym import ActionWrapper
import gym
from kicker.ai.rotate_to_target import create_rotate_to_target_bot
from stable_baselines3.common.callbacks import CheckpointCallback
from kicker.wrappers import FlattenAction
from kicker.callbacks import GoalCallback
from stable_baselines3.common.logger import configure
from kicker.wrappers import FlattenAction
from kicker.callbacks import GoalCallback
from stable_baselines3.common.vec_env import DummyVecEnv, SubprocVecEnv
import os
from typing import Callable
from stable_baselines3.common.utils import set_random_seed
from gym.wrappers import TimeLimit
from datetime import datetime


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
                    ai_function=create_rotate_to_target_bot,
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
    # num_cpu = 12  # Number of processes to use
    # # Create the vectorized environment
    # env = SubprocVecEnv([make_env(i) for i in range(num_cpu)])
    env = make_env(0)()
    timestamp = datetime.now().strftime("%Y-%m-%d.%H-%M-%S")

    model_dir = f"model/ppo_{timestamp}"
    if not os.path.exists(model_dir):
        os.makedirs(model_dir)

    logdir = f"logs/ppo_{timestamp}"
    if not os.path.exists(logdir):
        os.makedirs(logdir)

    checkpoint_callback = CheckpointCallback(
        save_freq=100000, save_path=logdir, name_prefix="ppo"
    )
    goal_callback = GoalCallback()
    new_logger = configure(logdir, ["csv", "tensorboard"])

    model = PPO("MlpPolicy", env, verbose=1, tensorboard_log="./cartpole_tensorboard/")
    model.set_logger(new_logger)
    model.learn(
        total_timesteps=10e6,
        log_interval=1,
        callback=[goal_callback, checkpoint_callback],
    )

    env.close()


if __name__ == "__main__":
    main()

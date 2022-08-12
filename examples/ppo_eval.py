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
from stable_baselines3.common.evaluation import evaluate_policy


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
                    bullet_connection_type=pb.GUI,
                    max_steps=10000,
                    ai_function=create_rotate_to_target_bot,
                )
            )
        )
        env = FrameStack(env, 2)
        env.seed(seed + rank)
        return env

    set_random_seed(seed)
    return _init


def main():
    env = make_env(0)()
    model = PPO("MlpPolicy", env, verbose=1)
    model.load("./model/ppo/")
    mean_reward, std_reward = evaluate_policy(model, env, n_eval_episodes=10)
    print(f'Mean reward: {mean_reward} +/- {std_reward:.2f}')
    env.close()


if __name__ == "__main__":
    main()

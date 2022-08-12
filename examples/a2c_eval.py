#!/usr/bin/env python3

from kicker.gym_env import KickerEnv
import pybullet as pb
from kicker.reset_functions import camera_reset
from stable_baselines3 import A2C
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
import sys

from a2c_train import make_env


def main():
    env = make_env(0, bullet_connection_type=pb.GUI)()
    model = A2C("MlpPolicy", env, verbose=1)
    model_path = sys.argv[1]
    print(model_path)
    # model.load(model_path)
    mean_reward, std_reward = evaluate_policy(model, env, deterministic=False, n_eval_episodes=10)
    print(f'Mean reward: {mean_reward} +/- {std_reward:.2f}')
    env.close()


if __name__ == "__main__":
    main()

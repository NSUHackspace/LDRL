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
import os


def main():
    # creating Gym environment
    env = FlattenAction(
        FlattenObservation(
            KickerEnv(
                bullet_connection_type=pb.DIRECT,
                max_steps=10000,
                ai_function=create_rotate_to_target_bot,
            )
        )
    )
    env = FrameStack(env, 2)

    model_dir = "model/ppo"
    if not os.path.exists(model_dir):
        os.makedirs(model_dir)

    logdir = "logs/ppo"
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

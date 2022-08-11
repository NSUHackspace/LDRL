#!/usr/bin/env python3

from kicker.gym_env import KickerEnv
import pybullet as pb
from kicker.reset_functions import camera_reset
from stable_baselines3 import PPO
from gym.wrappers import FlattenObservation
from gym.wrappers import FrameStack
from gym import ActionWrapper
import gym
from gym import spaces
from kicker.ai.rotate_to_target import create_rotate_to_target_bot


class FlattenAction(ActionWrapper):
    """Action space wrapper that flattens the actions."""

    def __init__(self, env: gym.Env):
        """Flattens the observations of an environment.

        Args:
            env: The environment to apply the wrapper
        """
        super().__init__(env)
        self.action_space = spaces.flatten_space(env.action_space)

    def action(self, action):
        """Flattens an action.

        Args:
            action: The action to flatten

        Returns:
            The flattened action
        """
        return spaces.unflatten(self.env.action_space, action)


def main():
    # creating Gym environment
    env = FlattenAction(
        FlattenObservation(
            KickerEnv(bullet_connection_type=pb.GUI, max_steps=10000,
            ai_function=create_rotate_to_target_bot)
        )
    )
    env = FrameStack(env, 2)

    model = PPO("MlpPolicy", env, verbose=1)
    model.learn(total_timesteps=1e6)

    obs = env.reset()
    for i in range(1000):
        action, _states = model.predict(obs, deterministic=True)
        obs, reward, done, info = env.step(action)
        env.render()
        if done:
            obs = env.reset()

    env.close()


if __name__ == "__main__":
    main()

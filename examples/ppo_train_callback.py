#!/usr/bin/env python3

from kicker.gym_env import KickerEnv
import pybullet as pb
from kicker.reset_functions import camera_reset
from stable_baselines3 import PPO
from gym.wrappers import FlattenObservation
from gym import ActionWrapper
import gym
from gym import spaces
from kicker.reward_functions import simple_reward
import os

from stable_baselines3.common.callbacks import BaseCallback

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

class GoalCallback(BaseCallback):
    def __init__(self, save_freq: int, save_path: str, name_prefix='goal', verbose=0):
        super(GoalCallback, self).__init__(verbose)
        self.save_freq = save_freq
        self.save_path = save_path
        self.name_prefix = name_prefix
        self.goals = 0

    def _init_callback(self) -> None:
        # Create folder if needed
        if self.save_path is not None:
            os.makedirs(self.save_path, exist_ok=True)
            filename = os.path.join(self.save_path, '{}.txt'.format(self.name_prefix))    

    
    def _on_step(self) -> bool:
        if self.training_env.get_attr("ball_return") != [0]:
            print(self.training_env.get_attr("ball_return"))
        return True


def main():


    # creating Gym environment
    env = FlattenAction(
        FlattenObservation(
            KickerEnv(bullet_connection_type=pb.GUI, ai_function=None, max_steps=5000, reward_function=simple_reward)
        )
    )

    callback = GoalCallback(1000, "logs", verbose=1)

    model = PPO("MlpPolicy", env, verbose=1)
    model.learn(total_timesteps=1E+6, callback=callback)

    # obs = env.reset()
    # for i in range(1000):
    #     action, _states = model.predict(obs, deterministic=True)
    #     obs, reward, done, info = env.step(action)
    #     env.render()
    #     if done:
    #         obs = env.reset()

    env.close()


if __name__ == "__main__":
    main()

from kicker.gym_env import KickerEnv
import pybullet as pb
from kicker.reset_functions import camera_reset
from stable_baselines3 import A2C
from gym.wrappers import FlattenObservation
from gym import ActionWrapper
import gym
from gym import spaces
import os
from stable_baselines3.common.logger import configure
from stable_baselines3.common.callbacks import CheckpointCallback
from stable_baselines3.common.callbacks import BaseCallback

from kicker.reward_functions import advanced_reward
from kicker.reward_functions import simple_reward




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
    def __init__(self, verbose=0):
        super(GoalCallback, self).__init__(verbose)
        self.win_goals = 0
        self.lose_goals = 0

    def _on_step(self) -> bool:
        res = self.training_env.get_attr("ball_return")
        if res == [1]:
            self.win_goals+=1
        elif res == [-1]:
            self.lose_goals+=1
        self.logger.record("win_goals", self.win_goals)
        self.logger.record("lose_goals", self.lose_goals)
        return True

def main():
    # creating Gym environment
    model_dir = "model/a2c"
    if not os.path.exists(model_dir):
        os.makedirs(model_dir)

    logdir = "logs/a2c"
    if not os.path.exists(logdir):
        os.makedirs(logdir)

    checkpoint_callback = CheckpointCallback(save_freq=1000, save_path=model_dir,
                                                 name_prefix='a2c')
    goal_callback = GoalCallback()

    new_logger = configure(logdir, ["csv", "tensorboard"])

    env = FlattenAction(
        FlattenObservation(
            KickerEnv(bullet_connection_type=pb.GUI, ai_function=None, max_steps=10000, reward_function=simple_reward)
        )
    )

    model = A2C("MlpPolicy", env, verbose=1)
    model.set_logger(new_logger)

    
    timesteps = 10000
    model.learn(total_timesteps=timesteps, log_interval=1, callback=[goal_callback, checkpoint_callback])

    env.close()


if __name__ == "__main__":
    main()

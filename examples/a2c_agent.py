from kicker.gym_env import KickerEnv
import pybullet as pb
from kicker.reset_functions import camera_reset
from stable_baselines3 import A2C
from gym.wrappers import FlattenObservation
import gym
from gym import spaces
import os
from stable_baselines3.common.logger import configure
from stable_baselines3.common.callbacks import CheckpointCallback
from kicker.reward_functions import advanced_reward
from kicker.reward_functions import simple_reward
from kicker.wrappers import FlattenAction
from kicker.callbacks import GoalCallback


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
            KickerEnv(bullet_connection_type=pb.GUI, ai_function=None, reward_function=simple_reward)
        )
    )
    env = TimeLimit(env, max_episode_steps=10000)

    model = A2C("MlpPolicy", env, verbose=1)
    model.set_logger(new_logger)


    timesteps = 10000
    model.learn(total_timesteps=timesteps, log_interval=1, callback=[goal_callback, checkpoint_callback])

    env.close()


if __name__ == "__main__":
    main()

from kicker.gym_env import KickerEnv
import pybullet as pb
from numpngw import write_apng
import numpy as np


# creating Gym environment
env = KickerEnv(bullet_connection_type=pb.DIRECT, render_mode="rgba_array")

env.reset()
start_img = env.render()
start_img = [np.array(start_img[0], dtype="uint8").reshape((800, 1024, 4))]

# rgba = np
write_apng("start.png", start_img, delay=20)

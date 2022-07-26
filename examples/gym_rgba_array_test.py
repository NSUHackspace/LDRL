from kicker.gym_env import KickerEnv
import pybullet as pb
from numpngw import write_apng
import numpy as np


# resolution "parameter"
resolution = (600, 400)

# creating Gym environment
env = KickerEnv(
    bullet_connection_type=pb.DIRECT,
    render_mode="rgba_array",
    render_resolution=resolution,
)

env.reset()
start_img = env.render()
start_img = [np.array(start_img, dtype="uint8").reshape((*resolution[::-1], 4))]

# rgba = np
write_apng("start.png", start_img, delay=20)

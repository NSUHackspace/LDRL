from src.kicker.gym_env import KickerEnv
from pybullet import getKeyboardEvents
from src.kicker.reset_functions import camera_reset


env = KickerEnv()

KEY_R = ord("r")
KEY_C = ord("c")

while True:
    getKeyboardEvents(env.pb_connection)
    keys = None
    env.reset()
    done = False
    for _ in " " * 10000:
        keys = getKeyboardEvents(env.pb_connection)
        action_space, rw, done, _ = env.step(env.action_space.sample())
        if done or KEY_R in keys:
            break
        if KEY_C in keys:
            camera_reset(env.pb_connection)
        # time.sleep(.05)

from kicker.gym_env import KickerEnv
from pybullet import getKeyboardEvents
from kicker.reset_functions import camera_reset

# hotkeys to reset environment and camera
KEY_R = ord("r")
KEY_C = ord("c")

while True:
    # reseting keys
    getKeyboardEvents(env.pb_connection)
    keys = None
    
    # reseting environment
    env.reset()
    done = False
    for _ in " " * 10000:
        # getting pressed keys
        keys = getKeyboardEvents(env.pb_connection)
        
        # environment step with random actions
        action_space, rw, done, _ = env.step(env.action_space.sample())

        # checking pressed keys
        if done or KEY_R in keys:
            break
        if KEY_C in keys:
            camera_reset(env.pb_connection)
        # time.sleep(.05)

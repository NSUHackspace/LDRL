import time

from kicker.utils.keyboard_binder import bind_arm, update_arms
from kicker.scene import create_scene
from kicker.reset_functions import *
from kicker.ai import simple_bot
import pybullet as pb
from pybullet import *


# initialize client
pb_client = pb.connect(pb.GUI)
# reset camera
resetDebugVisualizerCamera(cameraDistance=20, cameraYaw=90,
                           cameraPitch=-89.99999,
                           cameraTargetPosition=(0, 0, 0))
configureDebugVisualizer(COV_ENABLE_KEYBOARD_SHORTCUTS, 0)
configureDebugVisualizer(COV_ENABLE_GUI, 0)
configureDebugVisualizer(COV_ENABLE_RENDERING, 1)

# get object and start state(recovery point) from scene
objects, zero_state = create_scene(pb_client)

gme_board, gme_ply1_arm1, gme_ply1_arm2, gme_ply2_arm1, gme_ply2_arm2, gme_ball = objects.values()

# debug bindings
ply1_arm1_ctrl = bind_arm(gme_ply1_arm1, B3G_UP_ARROW, B3G_DOWN_ARROW,
                          B3G_RIGHT_ARROW, B3G_LEFT_ARROW)
ply1_arm2_ctrl = bind_arm(gme_ply1_arm2, ord("k"), ord("m"), ord(","), ord("n"))
ply2_arm1_ctrl = bind_arm(gme_ply2_arm1, ord(";"), ord("p"), ord("l"),
                          ord("'"))
ply2_arm2_ctrl = bind_arm(gme_ply2_arm2, ord("g"), ord("t"), ord("f"), ord("h"))

KEY_Q = ord('q')
KEY_R = ord('r')
KEY_C = ord('c')


bot_f = simple_bot(objects)

while True:

    ball_crds = getBasePositionAndOrientation(
                objects['ball'],
            )[0]
    # get all pressed keys from previous call
    keys = getKeyboardEvents()
    if KEY_Q in keys:
        break
    if KEY_R in keys:
        scene_reset(zero_state,
                    (
                        gme_ply1_arm1, gme_ply2_arm1, gme_ply2_arm2,
                        gme_ply1_arm2),
                    pb_client)
    if KEY_C in keys:
        camera_reset(pb_client)
    # handle key press
    update_arms(keys)
    bot_f(ball_crds)
    stepSimulation()
    time.sleep(1 / 240)

pb.disconnect()

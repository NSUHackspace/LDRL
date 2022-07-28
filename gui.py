import pybullet
from pybullet import *
import time
from scene import create_scene
from keyboard_binder import bind_arm, update_arms
from scene_functions import *

pb_client = pybullet.connect(pybullet.GUI)
params = getDebugVisualizerCamera()
resetDebugVisualizerCamera(cameraDistance=20, cameraYaw=90,
                           cameraPitch=-89.99999,
                           cameraTargetPosition=(0, 0, 0))
configureDebugVisualizer(COV_ENABLE_KEYBOARD_SHORTCUTS, 0)
configureDebugVisualizer(COV_ENABLE_GUI, 0)
configureDebugVisualizer(COV_ENABLE_RENDERING, 1)

objects, zero_state = create_scene(pb_client)

gme_board, gme_ply1_arm1, gme_ply1_arm2, gme_ply2_arm1, gme_ply2_arm2, gme_ball = objects.values()

ply1_arm1_ctrl = bind_arm(gme_ply1_arm1, B3G_UP_ARROW, B3G_DOWN_ARROW,
                          B3G_RIGHT_ARROW, B3G_LEFT_ARROW)
ply1_arm2_ctrl = bind_arm(gme_ply1_arm2, ord("k"), ord("m"), ord(","), ord("n"))
ply2_arm1_ctrl = bind_arm(gme_ply2_arm1, ord(";"), ord("p"), ord("l"),
                          ord("'"))
ply2_arm2_ctrl = bind_arm(gme_ply2_arm2, ord("g"), ord("t"), ord("f"), ord("h"))

KEY_Q = ord('q')
KEY_R = ord('r')
KEY_C = ord('c')

while True:
    keys = getKeyboardEvents()
    if KEY_Q in keys:
        break
    if KEY_R in keys:
        reset(zero_state,
              (gme_ply1_arm1, gme_ply2_arm1, gme_ply2_arm2, gme_ply1_arm2),
              pb_client)
    if KEY_C in keys:
        camera_reset(pb_client)
    update_arms(keys)
    stepSimulation()
    time.sleep(1 / 240)

pybullet.disconnect()

import pybullet
from pybullet import *
import time
import pybullet_data

pb_client = pybullet.connect(pybullet.GUI)
params = getDebugVisualizerCamera()
resetDebugVisualizerCamera(cameraDistance=20, cameraYaw=90, cameraPitch=-89.99999, cameraTargetPosition=(0,0,0))
configureDebugVisualizer(COV_ENABLE_KEYBOARD_SHORTCUTS, 0)
configureDebugVisualizer(COV_ENABLE_GUI, 0)
configureDebugVisualizer(COV_ENABLE_RENDERING, 1)


from scene import *
from keyboard_binder import bind_arm, update_arms


ply1_arm1_ctrl = bind_arm(gme_ply1_arm1, B3G_LEFT_ARROW, B3G_RIGHT_ARROW,
                          B3G_UP_ARROW, B3G_DOWN_ARROW)
ply1_arm2_ctrl = bind_arm(gme_ply1_arm2, ord("a"), ord("d"), ord("w"), ord("x"))
ply2_arm1_ctrl = bind_arm(gme_ply2_arm1, ord(";"), ord("\\"), ord("["),
                          ord("'"))
ply2_arm2_ctrl = bind_arm(gme_ply2_arm2, ord("f"), ord("h"), ord("t"), ord("g"))

KEY_Q = ord('q')
KEY_R = ord('r')

start_state = saveState()

while True:
    keys = getKeyboardEvents()
    if KEY_Q in keys:
        break
    if KEY_R in keys:
        restoreState(start_state)
    update_arms(keys)
    stepSimulation()
    time.sleep(1 / 240)

pybullet.disconnect()

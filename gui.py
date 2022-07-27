import pybullet
from pybullet import *
import time

pb_client = pybullet.connect(pybullet.GUI)
params = getDebugVisualizerCamera()
resetDebugVisualizerCamera(cameraDistance=20, cameraYaw=90,
                           cameraPitch=-89.99999,
                           cameraTargetPosition=(0, 0, 0))
configureDebugVisualizer(COV_ENABLE_KEYBOARD_SHORTCUTS, 0)
configureDebugVisualizer(COV_ENABLE_GUI, 0)
configureDebugVisualizer(COV_ENABLE_RENDERING, 1)

from scene import *
from keyboard_binder import bind_arm, update_arms

ply1_arm1_ctrl = bind_arm(gme_ply1_arm1, B3G_UP_ARROW, B3G_DOWN_ARROW,
                          B3G_RIGHT_ARROW, B3G_LEFT_ARROW)
ply1_arm2_ctrl = bind_arm(gme_ply1_arm2, ord("k"), ord("m"), ord(","), ord("n"))
ply2_arm1_ctrl = bind_arm(gme_ply2_arm1, ord(";"), ord("p"), ord("l"),
                          ord("'"))
ply2_arm2_ctrl = bind_arm(gme_ply2_arm2, ord("g"), ord("t"), ord("f"), ord("h"))

KEY_Q = ord('q')
KEY_R = ord('r')


def is_win():
    x, y, z = getBasePositionAndOrientation(gme_ball)[0]
    if y > 13 and z < 5:
        return 1
    if x < 0 or y < -13:
        return -1
    return 0


def reset_arm(arm_id):
    setJointMotorControl2(
        arm_id,
        2,
        POSITION_CONTROL,
        targetPosition=0,
    )
    setJointMotorControl2(
        arm_id,
        1,
        TORQUE_CONTROL,
        targetPosition=0,
        force=0
    )


while True:
    keys = getKeyboardEvents()
    if KEY_Q in keys:
        break
    if KEY_R in keys:
        restoreState(zero_state)
        for arm in (gme_ply1_arm2, gme_ply1_arm1, gme_ply2_arm1, gme_ply2_arm2):
            reset_arm(arm)
    print(is_win())
    update_arms(keys)
    stepSimulation()
    time.sleep(1 / 240)

pybullet.disconnect()

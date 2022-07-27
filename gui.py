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


def bind_arm(arm_id: int, left_key: int, right_key: int, up_key: int,
             down_key: int):
    rotator_id, slider_id = 1, 2

    def __check_function(keys):
        nonlocal arm_id, left_key, right_key, up_key, down_key, rotator_id, slider_id
        rotate_pos, slider_pos = 0, 0
        if right_key in keys:
            slider_pos = 1
        if left_key in keys:
            slider_pos = -1
        if up_key in keys:
            rotate_pos = 1
        if down_key in keys:
            rotate_pos = -1
        if slider_pos:
            # setJointMotorControl2(
            #     arm_id,
            #     slider_id,
            #     POSITION_CONTROL,
            #     targetPosition=slider_pos * 3,
            #     positionGain=.05
            # )

            slider = getJointState(arm_id, slider_id)
            setJointMotorControl2(
                arm_id,
                slider_id,
                POSITION_CONTROL,
                targetPosition=slider_pos * .5 + slider[0],
            )
        if rotate_pos:
            setJointMotorControl2(
                arm_id,
                rotator_id,
                TORQUE_CONTROL,
                force=1000 * rotate_pos
            )

    return __check_function


ply1_arm1_ctrl = bind_arm(gme_ply1_arm1, B3G_LEFT_ARROW, B3G_RIGHT_ARROW,
                          B3G_UP_ARROW, B3G_DOWN_ARROW)
ply1_arm2_ctrl = bind_arm(gme_ply1_arm2, ord("a"), ord("d"), ord("w"), ord("x"))
ply2_arm1_ctrl = bind_arm(gme_ply2_arm1, ord(";"), ord("\\"), ord("["),
                          ord("'"))
ply2_arm2_ctrl = bind_arm(gme_ply2_arm2, ord("f"), ord("h"), ord("t"), ord("g"))

KEY_Q = ord('q')
KEY_R = ord('r')

start_state = saveState()
# setRealTimeSimulation(1)
while True:
    keys = getKeyboardEvents()
    if KEY_Q in keys:
        break
    if KEY_R in keys:
        restoreState(start_state)
    ply1_arm1_ctrl(keys)
    ply1_arm2_ctrl(keys)
    ply2_arm1_ctrl(keys)
    ply2_arm2_ctrl(keys)
    stepSimulation()
    time.sleep(1 / 240)

pybullet.disconnect()

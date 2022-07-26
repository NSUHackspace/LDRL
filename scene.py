import pybullet
from pybullet import *
import time
import pybullet_data
from math import *


pb_client = pybullet.connect(pybullet.GUI)


pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
setGravity(0, 0, -9.8)

gme_board = loadURDF("./models/board.urdf", useFixedBase=1)

reversed_angle = [0, 0, 1, 0]

def gme_create_player(urdf_path: str, pos: (int, int, int), angle=(0, 0, 0, 1)) -> int:
    pb_id = loadURDF(urdf_path, pos, angle, useFixedBase=0)
    createConstraint(
        gme_board,
        0,
        pb_id,
        0,
        JOINT_FIXED,
        (0,0,0),
        (0,0,0),
        (0,0,0)
    )
    return pb_id


gme_ply1_arm1 = gme_create_player("./models/arm1.urdf", (23.25, -9.5, 3.5))
gme_ply1_arm2 = gme_create_player("./models/arm1.urdf", (23.25, 3.5, 3.5))

gme_ply2_arm2 = gme_create_player("./models/arm2.urdf", (-23.25, -3.5, 3.5),
                         reversed_angle)
gme_ply2_arm1 = gme_create_player("./models/arm2.urdf", (-23.25, 9.5, 3.5),
                         reversed_angle)

gme_ball = createMultiBody(
    baseMass=1,
    baseVisualShapeIndex=createCollisionShape(
        shapeType=GEOM_SPHERE,
        radius=1.25,
    ),
    baseCollisionShapeIndex=createVisualShape(
        shapeType=GEOM_SPHERE,
        radius=1.25,
        rgbaColor=(.7, .7, .7, 1)
    ),
    basePosition=(0, 0, 1.5),
)


def bind_arm(arm_id: int, left_key: int, right_key: int, up_key: int, down_key: int):
    rotator, slider = getJointStates(arm_id, (0, 1))

    def __check_function(keys):
        nonlocal  arm_id, left_key, right_key, up_key, down_key, rotator, slider
        rotate_pos, slider_pos = 0, 0
        if right_key in keys:
            slider_pos = -2
        if left_key in keys:
            slider_pos = 2
        if up_key in keys:
            rotate_pos = 2
        if down_key in keys:
            rotate_pos = -2
        if rotate_pos or slider_pos:
            print(f"-------- {rotate_pos} {slider_pos} <{rotator}> <{slider}>")
            setJointMotorControlArray(
                arm_id,
                (0, 1),
                POSITION_CONTROL,
                targetPositions=(rotator[0] + rotate_pos, slider[0] + slider_pos),
                targetVelocities=(0, 0),
                # forces=(1000, 1000),
                positionGains=(rotate_pos, slider_pos),
                # velocityGains=(rotate_pos, slider_pos)
            )
    return __check_function


arm1_ctrl = bind_arm(gme_ply1_arm1, ord('4'), ord('6'), ord('8'), ord('2'))


KEY_Q = ord('q')
# setRealTimeSimulation(1)
while True:
    keys = getKeyboardEvents()
    if KEY_Q in keys:
        break
    arm1_ctrl(keys)
    stepSimulation()
    time.sleep(1/240)

pybullet.disconnect()

import pybullet
from pybullet import *
import time
import pybullet_data
from math import *

pb_client = pybullet.connect(pybullet.GUI)
pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
setGravity(0, 0, -9.8)

gme_board = loadURDF("./models/board.urdf")

reversed_angle = [0, 0, 1, 0]


def gme_create_player(urdf_path: str, pos: (int, int, int), angle=(0, 0, 0, 1)) -> int:
    pb_id = loadURDF(urdf_path, pos, angle)
    print(setJointMotorControlArray(
        pb_id,
        (0, 1),
        controlMode=POSITION_CONTROL,
        targetPositions=(0, 0),
        targetVelocities=(0, 0),
    ))
    return pb_id


gme_ply1_arm1 = gme_create_player("./models/arm1.urdf", (23.25, -9.5, 3.5))
gme_ply1_arm2 = gme_create_player("./models/arm1.urdf", (23.25, 3.5, 3.5))

gme_ply2_arm2 = gme_create_player("./models/arm2.urdf", (-23.25, -3.5, 3.5),
                         reversed_angle)
gme_ply2_arm1 = gme_create_player("./models/arm2.urdf", (-23.25, 9.5, 3.5),
                         reversed_angle)

gme_ball = createMultiBody(
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

setRealTimeSimulation(1)
input()
pybullet.disconnect()

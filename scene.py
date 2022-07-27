import pybullet
from pybullet import *
import pybullet_data
from math import *


pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
setGravity(0, 0, -9.8)

gme_board = loadURDF("./models/board.urdf", useFixedBase=1)

reversed_angle = [0, 0, 1, 0]


def gme_create_player(urdf_path: str, pos: (int, int, int), angle=(0, 0, 0, 1)) -> int:
    pb_id = loadURDF(urdf_path, pos, angle, useFixedBase=0, )
    createConstraint(
        gme_board,
        0,
        pb_id,
        0,
        JOINT_FIXED,
        (0, 0, 0),
        pos,
        (0, 0, 0),
        (0, 0, 0, 1),
        angle,
    )
    return pb_id


gme_ply1_arm1 = gme_create_player("./models/arm1.urdf", (23.25, -9.5, 3.25))
gme_ply1_arm2 = gme_create_player("./models/arm1.urdf", (23.25, 3.5, 3.25))

gme_ply2_arm2 = gme_create_player("./models/arm2.urdf", (-23.25, -3.5, 3.25),
                         reversed_angle)
gme_ply2_arm1 = gme_create_player("./models/arm2.urdf", (-23.25, 9.5, 3.25),
                         reversed_angle)

gme_ball = createMultiBody(
    baseMass=5,
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

zero_state = saveState()

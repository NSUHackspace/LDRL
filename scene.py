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

gme_ply1_arm1 = loadURDF("./models/arm1.urdf", (23.25, -9.5, 3.5))
gme_ply1_arm2 = loadURDF("./models/arm1.urdf", (23.25, 3.5, 3.5))

gme_ply2_arm2 = loadURDF("./models/arm2.urdf", (-23.25, -3.5, 3.5),
                         reversed_angle)
gme_ply2_arm1 = loadURDF("./models/arm2.urdf", (-23.25, 9.5, 3.5),
                         reversed_angle)

ball_collision = createCollisionShape(
    shapeType=GEOM_SPHERE,
    radius=1.25,

)

ball_visual = createVisualShape(
    shapeType=GEOM_SPHERE,
    radius=1.25,
    rgbaColor=(.7, .7, .7, 1)
)

gme_ball = createMultiBody(
    baseVisualShapeIndex=ball_visual,
    baseCollisionShapeIndex=ball_collision,
    basePosition=(0, 0, 1.5),
)

input()

pybullet.disconnect()

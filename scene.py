import pybullet
from pybullet import *
import pybullet_data
from math import *

start_state = int


def create_scene(physicsClientId: int = 0) -> tuple[dict[str, int], start_state]:
    objects = dict()

    pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
    setGravity(0, 0, -9.8, physicsClientId=physicsClientId)

    objects['board'] = loadURDF("./models/board.urdf", useFixedBase=1,
                                physicsClientId=physicsClientId)

    reversed_angle = [0, 0, 1, 0]

    def gme_create_player(urdf_path: str, pos: (int, int, int),
                          angle=(0, 0, 0, 1)) -> int:
        pb_id = loadURDF(urdf_path, pos, angle, useFixedBase=0,
                         physicsClientId=physicsClientId)
        createConstraint(
            objects['board'],
            0,
            pb_id,
            0,
            JOINT_FIXED,
            (0, 0, 0),
            pos,
            (0, 0, 0),
            (0, 0, 0, 1),
            angle,
            physicsClientId=physicsClientId
        )
        return pb_id

    objects['player1_arm1'] = gme_create_player("./models/arm1.urdf",
                                                (23.25, -9.5, 3.25))
    objects['player1_arm2'] = gme_create_player("./models/arm1.urdf",
                                                (23.25, 3.5, 3.25))
    objects['player2_arm1'] = gme_create_player("./models/arm2.urdf",
                                                (-23.25, 9.5, 3.25),
                                                reversed_angle)
    objects['player2_arm2'] = gme_create_player("./models/arm2.urdf",
                                                (-23.25, -3.5, 3.25),
                                                reversed_angle)

    objects['ball'] = createMultiBody(
        baseMass=1,
        baseVisualShapeIndex=createCollisionShape(
            shapeType=GEOM_SPHERE,
            radius=1.00,
            physicsClientId=physicsClientId
        ),
        baseCollisionShapeIndex=createVisualShape(
            shapeType=GEOM_SPHERE,
            radius=1.00,
            rgbaColor=(.7, .7, .7, 1),
            physicsClientId=physicsClientId
        ),
        basePosition=(0, 0, 1.5),
        physicsClientId=physicsClientId
    )

    zero_state = saveState(physicsClientId)

    return objects, zero_state

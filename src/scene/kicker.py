import pybullet as pb
from pybullet import *
import pybullet_data
from typing import Tuple, Dict

start_state = int


def create_scene(physicsClientId: int = 0) -> Tuple[
    Dict[str, int], start_state]:
    """
    * loads all .urdf assets for kicker
    * creates ball
    * setups gravity

    :param physicsClientId: client id for multiple connections
    :return: dict of objects, start point to reset function
    """
    objects = dict()

    pb.setAdditionalSearchPath(pybullet_data.getDataPath())
    setGravity(0, 0, -9.8, physicsClientId=physicsClientId)

    objects['board'] = pb.loadURDF("./src/scene/assets/board.urdf",
                                   useFixedBase=1,
                                   physicsClientId=physicsClientId)

    reversed_angle = [0, 0, 1, 0]

    def gme_create_player(urdf_path: str,
                          pos: Tuple[float, float, float],
                          angle=(0., 0., 0., 1.)) -> int:
        """
        Loads the arm model and hardwires it to the static board

        :param urdf_path: path to urdf model
        :param pos: position relative to the center of coordinates
        :param angle: view angle in Quaternion form(?)
        :return: arms unique id
        """
        pb_id = pb.loadURDF(urdf_path, pos, angle, useFixedBase=0,
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

    objects['player1_arm1'] = gme_create_player("./src/scene/assets/arm1.urdf",
                                                (23.25, -9.5, 3.25))
    objects['player1_arm2'] = gme_create_player("./src/scene/assets/arm1.urdf",
                                                (23.25, 3.5, 3.25))
    objects['player2_arm1'] = gme_create_player("./src/scene/assets/arm2.urdf",
                                                (-23.25, 9.5, 3.25),
                                                reversed_angle)
    objects['player2_arm2'] = gme_create_player("./src/scene/assets/arm2.urdf",
                                                (-23.25, -3.5, 3.25),
                                                reversed_angle)

    # ball creation
    objects['ball'] = createMultiBody(
        baseMass=1,
        baseVisualShapeIndex=createCollisionShape(
            shapeType=pb.GEOM_SPHERE,
            radius=1.00,
            physicsClientId=physicsClientId
        ),
        baseCollisionShapeIndex=createVisualShape(
            shapeType=pb.GEOM_SPHERE,
            radius=1.00,
            rgbaColor=(.7, .7, .7, 1),
            physicsClientId=physicsClientId
        ),
        basePosition=(0, 0, 1.5),
        physicsClientId=physicsClientId
    )

    # saving created scene
    zero_state = pb.saveState(physicsClientId)

    return objects, zero_state

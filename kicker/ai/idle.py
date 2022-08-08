from pybullet import setJointMotorControl2
import pybullet as pb
from typing import Dict, Tuple


def create_idle_bot(pb_objects: Dict[str, int], physicsClientId: int = 0):
    arm1_id = pb_objects["player2_arm1"]
    arm2_id = pb_objects["player2_arm2"]
    rotator_id, slider_id = 1, 2

    def f(crds: Tuple[float, float, float]):
        setJointMotorControl2(
            arm1_id,
            rotator_id,
            pb.POSITION_CONTROL,
            targetPosition=3,
            force=1000,
            physicsClientId=physicsClientId
        )
        setJointMotorControl2(
            arm2_id,
            rotator_id,
            pb.POSITION_CONTROL,
            targetPosition=3,
            # force=1000,
            physicsClientId=physicsClientId
        )

    return f
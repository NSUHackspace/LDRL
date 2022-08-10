from pybullet import setJointMotorControl2
import pybullet as pb
from typing import Dict, Tuple
import numpy as np


def create_rotate_to_target_bot(pb_objects: Dict[str, int], physicsClientId: int = 0):
    arm1_id = pb_objects["player2_arm1"]
    arm2_id = pb_objects["player2_arm2"]
    rotator_id, slider_id = 1, 2

    def f(crds: Tuple[float, float, float]):
        x, y, z = crds
        setJointMotorControl2(
            arm1_id,
            rotator_id,
            pb.VELOCITY_CONTROL,
            targetVelocity=1.5,
            velocityGain=1,
            physicsClientId=physicsClientId,
        )
        setJointMotorControl2(
            arm2_id,
            rotator_id,
            pb.VELOCITY_CONTROL,
            targetVelocity=1.5,
            velocityGain=1,
            physicsClientId=physicsClientId,
        )

        arm1_pos = -x + np.random.uniform(-3, 3)
        setJointMotorControl2(
            arm1_id,
            slider_id,
            pb.POSITION_CONTROL,
            targetPosition=arm1_pos,
            positionGain=0.005,
            physicsClientId=physicsClientId,
        )

        arm2_pos = -x + np.random.uniform(-3, 3)
        setJointMotorControl2(
            arm2_id,
            slider_id,
            pb.POSITION_CONTROL,
            targetPosition=arm2_pos,
            positionGain=0.005,
            physicsClientId=physicsClientId,
        )

    return f


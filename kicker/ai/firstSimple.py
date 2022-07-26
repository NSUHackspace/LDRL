from pybullet import getLinkStates, setJointMotorControl2
import pybullet as pb
from typing import Dict, Tuple
from ..scene.consts import BALL_R, FOOTBALLER_SIZE_X


def create_bot(pb_objects: Dict[str, int], physicsClientId: int = 0):
    arm1_id = pb_objects["player2_arm1"]
    arm2_id = pb_objects["player2_arm2"]
    ball_id = pb_objects["ball"]
    rotator_id, slider_id = 1, 2
    # print(*getLinkStates(arm_id, (0,1,2,3,4,5)), sep="\n>")
    p1_a, p2_a, p3_a = map(lambda x: x[4], getLinkStates(arm1_id, (3, 4, 5),
                                                         physicsClientId=physicsClientId))
    p1_b, p2_b, p3_b = map(lambda x: x[4], getLinkStates(arm2_id, (3, 4, 5),
                                                         physicsClientId=physicsClientId))
    sx = 3
    p1ymi, p1yma = p1_a[1] - 3, p1_a[1] + .3
    p2ymi, p2yma = p1_b[1] - 4, p1_b[1] + 4
    force = 5000
    tick_count = 20

    def f(crds: Tuple[float, float, float]):
        nonlocal tick_count
        x, y, z = crds
        if p1ymi <= y <= p1yma and z < 3:
            setJointMotorControl2(
                arm1_id,
                rotator_id,
                pb.TORQUE_CONTROL,
                force=force,
                physicsClientId=physicsClientId
            )
        else:
            setJointMotorControl2(
                arm1_id,
                rotator_id,
                pb.TORQUE_CONTROL,
                force=-force,
                physicsClientId=physicsClientId
            )
        if p2ymi <= y <= p1_b[1] and z < 3 and tick_count < 100:
            setJointMotorControl2(
                arm2_id,
                rotator_id,
                pb.TORQUE_CONTROL,
                force=force,
                physicsClientId=physicsClientId
            )
            tick_count += 1
        elif p1_b[1] < y <= p2yma and z < 3 and tick_count < 100:
            setJointMotorControl2(
                arm2_id,
                rotator_id,
                pb.TORQUE_CONTROL,
                force=-force,
                physicsClientId=physicsClientId
            )
            tick_count += 1
        else:
            tick_count = 0
            setJointMotorControl2(
                arm2_id,
                rotator_id,
                pb.POSITION_CONTROL,
                targetPosition=0,
                force=force / 10,
                physicsClientId=physicsClientId
            )
        # sliders
        if x + FOOTBALLER_SIZE_X + BALL_R > p2_b[0]:
            setJointMotorControl2(
                arm2_id,
                slider_id,
                pb.POSITION_CONTROL,
                targetPosition=p2_b[0] - x,
                physicsClientId=physicsClientId
            )
        elif x < p3_b[0] + FOOTBALLER_SIZE_X + BALL_R:
            setJointMotorControl2(
                arm2_id,
                slider_id,
                pb.POSITION_CONTROL,
                targetPosition=p3_b[0] - x,
                physicsClientId=physicsClientId
            )
        else:
            setJointMotorControl2(
                arm2_id,
                slider_id,
                pb.POSITION_CONTROL,
                targetPosition=p1_b[0] - x,
                physicsClientId=physicsClientId
            )
        if x + FOOTBALLER_SIZE_X + BALL_R > p2_a[0]:
            if p1ymi <= y <= p1yma:
                setJointMotorControl2(
                    arm1_id,
                    slider_id,
                    pb.POSITION_CONTROL,
                    targetPosition=p2_a[0] - x,
                    physicsClientId=physicsClientId
                )
            else:
                setJointMotorControl2(
                    arm1_id,
                    slider_id,
                    pb.POSITION_CONTROL,
                    targetPosition=p2_a[0] - x,
                    physicsClientId=physicsClientId
                )
        elif x < p3_a[0] + FOOTBALLER_SIZE_X + BALL_R:
            setJointMotorControl2(
                arm1_id,
                slider_id,
                pb.POSITION_CONTROL,
                targetPosition=p3_a[0] - x,
                physicsClientId=physicsClientId
            )
        else:
            setJointMotorControl2(
                arm1_id,
                slider_id,
                pb.POSITION_CONTROL,
                targetPosition=p1_a[0] - x,
                physicsClientId=physicsClientId
            )

    return f

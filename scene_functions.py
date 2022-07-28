from pybullet import *
from typing import Iterable


def __reset_arm(arm_id: int, physicsClientId: int = 0):
    setJointMotorControl2(
        arm_id,
        2,
        POSITION_CONTROL,
        targetPosition=0,
        physicsClientId=physicsClientId,
    )
    setJointMotorControl2(
        arm_id,
        1,
        TORQUE_CONTROL,
        targetPosition=0,
        force=0,
        physicsClientId=physicsClientId,
    )


def is_win(ball_id: int, physicsClientId: int = 0):
    x, y, z = \
        getBasePositionAndOrientation(ball_id, physicsClientId=physicsClientId)[
            0]
    if y > 13 and z <= 5 and -3 < x < 3:
        return 1
    if y < -13 or z < 0 or (y > 13 and z > 5):
        return -1
    return 0


def reward_f(ball_id: int, physicsClientId: int = 0):
    def minus_mult(a, b):
        return a * b * -1 ** (a < 0 and b < 0)

    x, y, z = \
        getBasePositionAndOrientation(ball_id, physicsClientId=physicsClientId)[
            0]
    return minus_mult((9 - x ** 2) + minus_mult((90 - x ** 2), (6 - z)),
                      (y + 13))


def reset(zero_state: int, arms_id_list: Iterable[int],
          physicsClientId: int = 0):
    restoreState(zero_state, physicsClientId=physicsClientId)
    for arm_id in arms_id_list:
        __reset_arm(arm_id, physicsClientId=physicsClientId)


def camera_reset(physicsClientId: int = 0):
    resetDebugVisualizerCamera(cameraDistance=20, cameraYaw=90,
                               cameraPitch=-89.99999,
                               cameraTargetPosition=(0, 0, 0),
                               physicsClientId=physicsClientId
                               )


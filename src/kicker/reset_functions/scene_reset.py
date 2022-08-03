import pybullet as pb
from typing import Iterable


# Variables dependent on urdf models. Set globally for simplicity
__rotator_id = 1
__slider_id = 2


def _reset_arm(arm_id: int, physicsClientId: int = 0):
    """
    Sets certain arm to its base position

    :param arm_id: arm unique id
    :param physicsClientId: client id for multiple connections
    :return: None
    """
    pb.setJointMotorControl2(
        arm_id,
        __slider_id,
        pb.POSITION_CONTROL,
        targetPosition=0,
        physicsClientId=physicsClientId,
    )
    pb.setJointMotorControl2(
        arm_id,
        __rotator_id,
        pb.POSITION_CONTROL,
        targetPosition=0,
        force=0,
        physicsClientId=physicsClientId,
    )


def scene_reset(zero_state: int, arms_id_list: Iterable[int],
                physicsClientId: int = 0):
    """
    Resets scene to the starting state:

    * Resets ball position to 0, 0, 0
    * Resets the motors of the given arms to the starting point

    :param zero_state: the state returned by the create_scene(...) function
    :param arms_id_list: iterable of unique arm id
    :param physicsClientId: client id for multiple connections
    :return:
    """
    pb.restoreState(zero_state, physicsClientId=physicsClientId)
    for arm_id in arms_id_list:
        _reset_arm(arm_id, physicsClientId=physicsClientId)


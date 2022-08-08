from pybullet import *
import pybullet as pb
from typing import List, Callable

__binds: List[Callable] = []


def update_arms(keys):
    """
    Checks if the bound key was pressed and calls a function

    :param keys: iterable(?) of pressed keys
    :return:
    """
    for bind in __binds:
        bind(keys)


def bind_arm(arm_id: int, left_key: int, right_key: int, up_key: int,
             down_key: int):
    """
    Binds arm control to the given keys(dependent of the camera position)

    :param arm_id: arm unique id
    :param left_key: slider up key
    :param right_key: slider down key
    :param up_key: rotator right key
    :param down_key: rotator left key
    :return:
    """
    rotator_id, slider_id = 1, 2

    def __check_function(keys):
        """
        Checks if bind_arm args are in keys iter and changes the state of the motors

        :param keys: iterable of pressed keys
        :return:
        """
        nonlocal arm_id, left_key, right_key, up_key, down_key, rotator_id, slider_id
        rotate_pos, slider_pos = 0, 0
        if right_key in keys:
            slider_pos = 1
        if left_key in keys:
            slider_pos = -1
        if up_key in keys:
            rotate_pos = 1
        if down_key in keys:
            rotate_pos = -1
        if slider_pos:
            slider = getJointState(arm_id, slider_id)
            setJointMotorControl2(
                arm_id,
                slider_id,
                pb.POSITION_CONTROL,
                targetPosition=slider_pos * .5 + slider[0],
            )
        if rotate_pos:
            # print(getJointState(arm_id, rotator_id))
            setJointMotorControl2(
                arm_id,
                rotator_id,
                pb.TORQUE_CONTROL,
                force=2000 * rotate_pos
            )

    __binds.append(__check_function)
    return __check_function

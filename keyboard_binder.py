from pybullet import *

__binds: list[callable] = []


def update_arms(keys):
    for bind in __binds:
        bind(keys)


def bind_arm(arm_id: int, left_key: int, right_key: int, up_key: int,
             down_key: int):
    rotator_id, slider_id = 1, 2

    def __check_function(keys):
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
            # setJointMotorControl2(
            #     arm_id,
            #     slider_id,
            #     POSITION_CONTROL,
            #     targetPosition=slider_pos * 3,
            #     positionGain=.05
            # )

            slider = getJointState(arm_id, slider_id)
            setJointMotorControl2(
                arm_id,
                slider_id,
                POSITION_CONTROL,
                targetPosition=slider_pos * .5 + slider[0],
            )
        if rotate_pos:
            setJointMotorControl2(
                arm_id,
                rotator_id,
                TORQUE_CONTROL,
                force=1000 * rotate_pos
            )

    __binds.append(__check_function)
    return __check_function

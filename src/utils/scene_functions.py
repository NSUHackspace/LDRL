from pybullet import *
import pybullet as pb
from typing import Iterable, Tuple

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
    setJointMotorControl2(
        arm_id,
        __slider_id,
        pb.POSITION_CONTROL,
        targetPosition=0,
        physicsClientId=physicsClientId,
    )
    setJointMotorControl2(
        arm_id,
        __rotator_id,
        pb.POSITION_CONTROL,
        targetPosition=0,
        force=0,
        physicsClientId=physicsClientId,
    )


def is_done(ball_cds: Tuple[float, float, float]) -> int:
    """
    Checks if game is done.

    :param ball_cds: coordinates of the ball
    :return: > 1 if player 1 wins <br>
    > -1 if player 2 wins <br>
    > 0 if the ball went out of the playing area
    """
    x, y, z = ball_cds
    if y > 13 and z <= 5 and -3 < x < 3:
        return 1
    if y < -13 or z < 0 or (y > 13 and z > 5):
        return -1
    return 0


def simple_reward(ball_cds: Tuple[float, float, float], *args, **kwargs) -> float:
    """
    Some simple reward function realization.
    The idea is the more the function:

    * the closer ball to the player2's football goal
    * the closer ball to the center axis
    * the ball is on the board
    * The value of the function becomes negative when player1 loses or the ball falls out of the playing area

    :param ball_cds: coordinats of the ball
    :param *args: for compability
    :param **kwargs: for compability
    :return: reward value
    """

    def minus_mult(a: float, b: float) -> float:
        """
        Function that multiplies a and b, but changes sign of the result to minus if a < 0 and b < 0

        :param a: value1
        :param b: value2
        :return: value1 * value2 * (-1 if value1 < 0 and value2 < 0)
        """
        return a * b * -1 ** (a < 0 and b < 0)

    x, y, z = ball_cds
    return minus_mult((9 - x ** 2) + minus_mult((90 - x ** 2), (6 - z)),
                      (y + 13))


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


def camera_reset(physicsClientId: int = 0):
    """
    Sets the position of the camera - above the board; and direction of view - on the board

    :param physicsClientId: client id for multiple connections
    :return:
    """
    resetDebugVisualizerCamera(cameraDistance=20, cameraYaw=90,
                               cameraPitch=-89.99999,
                               cameraTargetPosition=(0, 0, 0),
                               physicsClientId=physicsClientId
                               )

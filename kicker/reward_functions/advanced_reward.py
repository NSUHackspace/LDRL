from typing import Tuple
from ..is_done_functions import is_done_by_coords
from ..scene.consts import *


def reward_function(ball_cds: Tuple[float, float, float], *args,
                    **kwargs) -> float:
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
    return minus_mult(
        ((BOARD_SIZE_X - BOARD_GATE_WALL_SIZE_X * 2) - x ** 2) + minus_mult(
            ((BOARD_SIZE_X - BOARD_GATE_WALL_SIZE_X * 2) ** 2 - x ** 2),
            (BOARD_SIZE_Z - z)),
        (y + BOARD_SHIFT_Z))


def simple_reward(ball_cds: Tuple[float, float, float], *args,
                  **kwargs) -> float:
    """
    Simplest reward function: just positive reward if agent win and negative if it lost

    :param ball_cds: coordinats of the ball
    :param *args: for compability
    :param **kwargs: for compability
    :return: reward value
    """

    if is_done_by_coords.is_done(ball_cds) != 0:
        return is_done_by_coords.is_done(ball_cds) * 200
    else:
        return -0.01

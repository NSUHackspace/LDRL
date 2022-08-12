from typing import Tuple
from ..scene.consts import *


GATE_HALF_SIZE_X = (BOARD_SIZE_X - BOARD_GATE_WALL_SIZE_X * 2) / 2


def is_done(ball_cds: Tuple[float, float, float]) -> int:
    """
    Checks if game is done.

    :param ball_cds: coordinates of the ball
    :return: > 1 if player 1 wins <br>
    > -1 if player 2 wins <br>
    > 0 if the ball went out of the playing area
    """
    x, y, z = ball_cds
    if y > BOARD_GATE_WALL_SHIFT_Y and z + BALL_R <= BOARD_SIZE_Z and -GATE_HALF_SIZE_X < x < GATE_HALF_SIZE_X:
        return 1
    if y < -BOARD_GATE_WALL_SHIFT_Y or z < 0 or (y > BOARD_GATE_WALL_SHIFT_Y and z > BOARD_SIZE_Z):
        return -1
    return 0

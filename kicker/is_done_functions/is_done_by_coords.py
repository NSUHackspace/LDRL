from typing import Tuple


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

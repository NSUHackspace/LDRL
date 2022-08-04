from typing import Tuple


def reward_function(ball_cds: Tuple[float, float, float], *args, **kwargs) -> float:
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

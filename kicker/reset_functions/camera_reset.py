from pybullet import resetDebugVisualizerCamera


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

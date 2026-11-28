import math


def rotate_pose_180(
    x: float, y: float, angle: float = 0.0
) -> tuple[float, float, float]:
    """Rotate a pose 180 degrees about the origin.

    Position is negated and the heading shifts by half a turn. Yaw rate is not
    part of this transform: a rotation about +z keeps its sign.

    :param x: The x coordinate, in metres
    :param y: The y coordinate, in metres
    :param angle: The heading, in radians
    :return: x, y, and heading after the half-turn
    """
    return -x, -y, angle + math.pi

from math import inf
from const import Coordinate
from controller.lidar_point import LidarPoint


def clamp(value: float, min_v: float, max_v: float):
    return min(max_v, max(value, min_v))


def point_cloud_filter(points: list[LidarPoint]) -> list[Coordinate]:
    new_points: list[Coordinate] = []
    for point in points:
        x, y, z = point.x, point.y, point.z
        if abs(x) == inf or abs(y) == inf or abs(z) == inf:
            continue
        new_points.append((x, y, z))
    return new_points


def print_help():
    print(
        """
You can control the drone with your computer keyboard:
    k:     move forward
    j:     move backward
    h:     turn right
    l:     turn left
    up:    increase the target altitude
    down:  decrease the target altitude
    right: strafe right
    left:  strafe left
"""
    )

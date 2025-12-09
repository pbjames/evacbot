# TODO: Rewrite with numpy
import heapq
import math
from random import random

from const import DRONE_BOUNDING_BOX, ORIGIN, SAMPLE_ATTEMPTS, BoundingBox, Coordinate


class Pathing:
    """
    We are dealing with masses of points which describe physical presence around us.
    Navigable points can be decided by choosing a random place around us that contains
    no points within the size of our bounding box.

    We can keep track of navigated points and calculate a travel weight based on where
    we've been before, what's dangerous and how risky it would be to navigate.
    """

    def __init__(self):
        self.__point_cloud: list[Coordinate] = []
        self.__current_position: Coordinate = ORIGIN

    @property
    def position(self):
        return self.__current_position

    @position.setter
    def position(self, value: Coordinate):
        # TODO: Do sanity checks or something
        self.__current_position = value

    @property
    def point_cloud(self):
        return self.__point_cloud

    def add_points(self, point_cloud: list[Coordinate]):
        self.__point_cloud.extend(point_cloud)

    def clear(self):
        self.__point_cloud.clear()

    def cloud_bounding_box(self) -> BoundingBox:
        max_x = max_y = max_z = -math.inf
        min_x = min_y = min_z = math.inf
        for x, y, z in self.__point_cloud:
            max_x, max_y, max_z = max(x, max_x), max(y, max_y), max(z, max_z)
            min_x, min_y, min_z = min(x, min_x), min(y, min_y), min(z, min_z)
        return (max_x, max_y, max_z), (min_x, min_y, min_z)

    def sample_random_point(self) -> Coordinate:
        """
        Sample from random linear interpolations through a bounding box in 3D space.

        Returns:
            Randomly generated coordinate
        """
        (max_x, max_y, max_z), (min_x, min_y, min_z) = self.cloud_bounding_box()
        x = min_x + random() * abs(max_x - min_x)
        y = min_y + random() * abs(max_y - min_y)
        z = min_z + random() * abs(max_z - min_z)
        return x, y, z

    @staticmethod
    def check_coordinate_within(u: Coordinate, bbox: BoundingBox):
        x, y, z = u
        (max_x, max_y, max_z), (min_x, min_y, min_z) = bbox
        return max_x >= x >= min_x, max_y >= y >= min_y, max_z >= z >= min_z

    def sample_safe_point(self) -> Coordinate:
        """
        Find a random point within the point cloud bounding box that has the clearance
        for the drone to safely move to that point, otherwise return our position.

        Returns:
            Coordinate of random point, or position if there is no apparent space.
        """
        (max_x, max_y, max_z), (min_x, min_y, min_z) = DRONE_BOUNDING_BOX
        pos_x, pos_y, pos_z = self.position
        max_coord: Coordinate = max_x + pos_x, max_y + pos_y, max_z + pos_z
        min_coord: Coordinate = (min_x + pos_x, min_y + pos_y, min_z + pos_z)
        bbox: BoundingBox = (max_coord, min_coord)
        attempts = 0
        while (
            not self.check_coordinate_within(point := self.sample_random_point(), bbox)
            and attempts <= SAMPLE_ATTEMPTS
        ):
            attempts += 1
        print(f"sample_safe_point() {bbox=} {point=} {attempts=}")
        return point if attempts <= SAMPLE_ATTEMPTS else self.position

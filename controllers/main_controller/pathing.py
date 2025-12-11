# TODO: Rewrite with numpy
import random

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
        p_x, p_y, p_z = self.position
        self.__point_cloud = [(p_x + x, p_y + y, p_z + z) for x, y, z in point_cloud]

    def clear(self):
        self.__point_cloud.clear()

    def sample_random_point(self) -> Coordinate:
        """
        Sample a random point from our point cloud and generate linear interpolation
        between that and our position. It's guaranteed by the sensor that there's no
        visible obstruction.

        Returns:
            Randomly generated coordinate
        """
        dst_x, dst_y, dst_z = random.choice(self.point_cloud)
        our_x, our_y, our_z = self.position
        x = our_x + random.random() * abs(dst_x - our_x)
        y = our_y + random.random() * abs(dst_y - our_y)
        z = our_z + random.random() * abs(dst_z - our_z)
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
        return point if attempts <= SAMPLE_ATTEMPTS else self.position

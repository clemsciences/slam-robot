from typing import List, Optional

import numpy as np
from scipy.optimize import root

from slam_robot.utils.geometry import Point


class Cluster:
    minimum_distance_between_clusters = 10  # in mm
    minimum_points_in_cluster = 3  # in mm
    maximum_distance_between_means = 20

    def __init__(self):
        self.points: List[Point] = []
        self.mean: Optional[Point] = None

    def append(self, point: Point):
        self.points.append(point)
        self.update_mean()

    def extend(self, other: 'Cluster'):
        self.points.extend(other.points)
        self.update_mean()

    def pop(self):
        point = self.points.pop()
        self.update_mean()
        return point

    def __len__(self):
        return len(self.points)

    def __iter__(self):
        return iter(self.points)

    def distance(self, other: 'Cluster'):
        """
        :param other:
        :return:
        """
        dist1 = self.points[0].distance(other.points[-1])
        dist2 = self.points[-1].distance(other.points[0])
        return min([dist1, dist2])

    def distance_to_point(self, point: Point):
        """
        Minimum distance to any point in the cluster.
        :param point:
        :return:
        """
        if self.points:
            minimum_distance = point.distance(self.points[0])
            for i in range(1, len(self.points)):
                distance = self.points[i].distance(point)
                if distance < minimum_distance:
                    minimum_distance = distance
            return minimum_distance
        return 0

    def update_mean(self):
        self.mean = Point.from_array(np.sum([point.to_array() for point in self.points], axis=0) / len(self.points))

    @property
    def x_points(self):
        return [i.x for i in self.points]

    @property
    def y_points(self):
        return [i.y for i in self.points]

    def is_a_circle(self, radius):
        def objective_function(pos):
            circle_position = pos
            dist_sum = 0
            for point in self.points:
                dist_sum += (point.distance(circle_position) - radius) ** 2
            return dist_sum, 0
        initial_guess = self.mean.to_array()
        solution = root(objective_function, initial_guess, method="lm")

        return solution

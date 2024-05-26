import imghdr
from typing import List, Optional

import numpy as np

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

        :param cluster:
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


class RobotPerception:
    def __init__(self,
                 timestamp: float,
                 obstacles: List[Optional[Point]],
                 position: Point,
                 ):
        self.timestamp = timestamp
        self.obstacles = obstacles
        self.position = position

    def clusterize(self) -> List[Cluster]:
        """
        :return:
        """
        clusters = []
        if len(self.obstacles) > 0:
            n = 0
            clusters.append(Cluster())
            clusters[0].append(self.obstacles[0])

            for i in range(1, len(self.obstacles) - 1):
                if self.obstacles[i - 1].distance(self.obstacles[i]) > Cluster.minimum_distance_between_clusters:
                    n += 1
                    clusters.append(Cluster())
                clusters[n].append(self.obstacles[i])
            if self.obstacles[0].distance(self.obstacles[-1]) <= Cluster.minimum_distance_between_clusters:
                # TODO fix it
                if len(clusters) > 1:
                    print(len(clusters))
                    clusters[-1].extend(clusters[0])
                    clusters[0] = clusters.pop()
                    n -= 1

            if len(clusters) > 1:
                j = 0
                k = 1
                while k < n:
                    dist_j = clusters[j].distance(clusters[k])
                    # if cluster barycenters are close enough to each other, then clusters are merged
                    if dist_j < Cluster.maximum_distance_between_means:
                        clusters[j].extend(clusters[k])
                        del clusters[k]
                        n -= 1
                    else:
                        # if the j'th and k'th are far enough, then, they are just different clusters
                        j += 1
                        k += 1
                        # if a cluster has too few points, then it is deleted
                        if len(clusters[j - 1]) < Cluster.minimum_points_in_cluster:
                            del clusters[j - 1]
                            n -= 1
                # if cluster_distance_mean(clusters[0], clusters[-1]) < 200:
                if clusters[0].distance(clusters[-1]) < Cluster.maximum_distance_between_means:
                    clusters[-1].extend(clusters[0])
                    clusters[0] = clusters.pop()
        return clusters

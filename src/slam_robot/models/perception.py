from typing import List, Optional

from slam_robot.models.cluster import Cluster
from slam_robot.utils.geometry import Point


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



import numpy as np

class Cluster:
    """
    Cluster of points. May be an obstacle or a beacon.
    """
    def __init__(self, beacon_radius, opponent_robot_radius):
        self.points = []
        self.mean = None
        self.beacon_radius = beacon_radius
        self.adverse_robot_radius = opponent_robot_radius
        self.closest_to_robot = None

    def get_std(self):
        return np.std(self.points)

    def is_linear(self):
        """
        Hough transform or RANSAC
        :return:
        """
        pass

    def compute_mean(self):
        if len(self.points) > 0:
            self.mean = np.sum(self.points, axis=0)/len(self.points)
        else:
            self.mean = np.array([0, 0])

    def get_mean(self):
        if len(self.points) > 0:
            self.compute_mean()
            return self.mean
        else:
            return None

    def add_point(self, point):
        self.points.append(point)

    def add_points(self, points):
        self.points.extend(points)

    def distance(self, other):
        if isinstance(other, Cluster):
            cluster_mean = np.sum(self.points, axis=0) / len(self.points)
            other_mean = np.sum(other.points, axis=0) / len(other.points)
            return distance(cluster_mean, other_mean)

    def add_cluster(self, other):
        self.points.extend(other)

    def __len__(self):
        return len(self.points)

    def is_a_fix_beacon(self):
        """

        >>> thetas = np.deg2rad(np.arange(0, 150, 6))
        >>> real_x, real_y = (-420, 780)
        >>> real_radius = 100
        >>> xx = real_radius * np.cos(thetas) + real_x
        >>> yy = real_radius * np.sin(thetas) + real_y

        # >>> import matplotlib.pyplot as plt
        # >>> l = plt.plot(xx, yy)

        >>> points = [np.array([xx[i], yy[i]]) for i in range(len(xx))]
        >>> cluster = Cluster()
        >>> cluster.add_points(points)
        >>> cluster.is_a_fix_beacon()

        :return:
        """

        initial_guess = self.get_mean()
        solution = root(self._beacons_objective_function, initial_guess, method="lm")

        beacon = None
        if np.isclose(solution.fun[0], 0, atol=TOLERANCE_FOR_CIRCLE_COHERENCE):
            beacon = Beacon()
            beacon.set_parameters(solution.x[0], solution.x[1], FIX_BEACON_RADIUS, 0)
            beacon.set_cluster(self)
        return beacon

    def is_an_opponent_robot_beacon(self):
        """
        Should check that the point is in table ?
        :return:
        """
        cluster_mean = self.get_mean()
        return cluster_mean

    def is_a_circle(self, radius):
        def objective_function(pos):
            circle_position = pos
            dist_sum = 0
            for point in self.points:
                dist_sum += (distance(point, circle_position) - radius) ** 2
            return dist_sum, 0
        initial_guess = self.get_mean()
        solution = root(objective_function, initial_guess, method="lm")

        return solution

    def _beacons_objective_function(self, pos):
        # circle_position = np.array([x, y])
        return self._objective_function(pos, self.beacon_radius)

    def _objective_function(self, pos, radius):
        circle_position = pos
        dist_sum = 0
        for point in self.points:
            dist_sum += (distance(point, circle_position) - radius) ** 2
        return dist_sum, 0

    def _adverse_objective_function(self, pos):
        return self._objective_function(pos, self.adverse_robot_radius)

    def new_cluster_by_points(self, points: List):
        new_cluster = Cluster()
        new_cluster.points = points
        return new_cluster

    @staticmethod
    def to_clusters(clusters, closest_points=None):
        """

        :param clusters: list of lists of 2D points
        :param closest_points:
        :return: list of clusters
        """
        real_clusters = []
        for i, cluster in enumerate(clusters):
            new_cluster = Cluster()
            if closest_points is not None:
                new_cluster.set_closest_to_robot(closest_points[i])
            new_cluster.add_points(cluster)
            real_clusters.append(new_cluster)
        return real_clusters

    def polar_to_cartesian(self):
        """

        :return:
        """
        self.points = [[outr.polar_to_x(point), outr.polar_to_y(point)] for point in self.points]

    def set_closest_to_robot(self, point):
        self.closest_to_robot = point

    def compute_closest_to_robot(self):
        min_dist = 4000
        closest_point = self.points[0]
        for point in self.points:
            current_norm = norm(point)
            if current_norm < min_dist:
                closest_point = point
                min_dist = current_norm
        self.set_closest_to_robot(closest_point)

    def get_closest_point_to_robot(self) -> np.ndarray:
        return np.array(self.closest_to_robot)

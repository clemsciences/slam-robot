
import slam_robot.utils.geometry as geom


class CylinderBeacon:
    def __init__(self):
        self.radius = 0
        self.x_center = 0
        self.y_center = 0
        self.center = None
        self.index = 0
        self.cluster = None

    def set_parameters(self, x: int, y: int, r: int, i: int):
        self.x_center = x
        self.y_center = y
        self.radius = r
        self.index = i
        self.center = geom.Point(x, y)

    def set_cluster(self, cluster):
        self.cluster = cluster

    def set_by_upper_left_and_lower_right(self, upper_left, lower_right):
        self.x_center = (upper_left[0]+lower_right[0])/2
        self.y_center = (upper_left[1]+lower_right[1])/2
        self.center = geom.Point(self.x_center, self.y_center)

    def set_radius(self, radius):
        self.radius = radius

    def set_index(self, index):
        self.index = index

    def __str__(self):
        return str(self.center)+" , "+str(self.radius)+" n°"+str(self.index)

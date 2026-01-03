import matplotlib.pyplot as plt

import slam_robot.utils.geometry as geom
from slam_robot.models.world_items import Circle


class CylinderBeacon(Circle):
    def __init__(self):
        super().__init__(geom.Point(0, 0), 0)
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

    def get_collision(self, origin: geom.Point, angle: float) -> list:
        return super().get_collision(origin, angle)

    def draw(self, ax, limit_inf_x=0, limit_sup_x=100, limit_inf_y=0, limit_sup_y=100, description=""):
        circle = plt.Circle(self.center.to_tuple(), self.radius, edgecolor="green", facecolor="none")
        ax.add_patch(circle)


    def __str__(self):
        return str(self.center)+" , "+str(self.radius)+" n°"+str(self.index)

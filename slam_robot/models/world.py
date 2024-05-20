from typing import List, Optional, Any

from slam_robot.models.world_items import WorldItem, LineByPointAndAngle
from slam_robot.utils.geometry import Point


class World:
    def __init__(self, items: List[WorldItem], limit_x: int, limit_y: int):
        self.items = items
        self.limit_x = limit_x
        self.limit_y = limit_y

    def see_obstacles(self, point: Point, angle: float) -> Optional[Point]:
        """
        Returns the distance at which the first obstacle visible from {self.items} at {angle} angle.

        :param point:
        :param angle:
        :return:
        """
        line = LineByPointAndAngle(point, angle)
        collisions = []
        for item in self.items:
            intersections = item.get_collision(point, angle)
            if intersections:
                collisions.extend(intersections)
        collision = None
        for c in collisions:
            # measure_vector = point.from_angle_to_vector(angle)
            # collision_vector = point.from_other_point_to_vector(c)
            # scalar_product = measure_vector.scalar_product(collision_vector)
            if line.is_in_same_sense(c):
                if collision:
                    if point.distance(c) < point.distance(collision):
                        collision = c
                else:
                    collision = c
        return collision
        # if collisions:
        #     if len(collisions) > 1:
        #         collision = None
        #         for c in collisions:
        #             # measure_vector = point.from_angle_to_vector(angle)
        #             # collision_vector = point.from_other_point_to_vector(c)
        #             # scalar_product = measure_vector.scalar_product(collision_vector)
        #             if line.is_in_same_sense(c):
        #                 if collision:
        #                     if point.distance(c) < point.distance(collision):
        #                         collision = c
        #                 else:
        #                     collision = c
        #
        #     else:
        #         if line.is_in_same_sense(collisions[0]):
        #             collision = collisions[0]
        #     return collision
        # return None

    def draw(self, ax: Any):
        ax.set_xlim([-10, self.limit_x+10])
        ax.set_ylim([-10, self.limit_y+10])
        # x_values = []
        # y_values = []
        for item in self.items:
            item.draw(ax, limit_inf_x=0, limit_sup_x=self.limit_x, limit_inf_y=0, limit_sup_y=self.limit_y)
            # x_values.append(point.x)
            # y_values.append(point.y)
        # plt.scatter(x_values, y_values)

        # plt.xlabel("X")
        # plt.ylabel("Y")
        # plt.title("Plot")
        # plt.show()





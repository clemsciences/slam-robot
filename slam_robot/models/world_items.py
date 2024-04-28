import math
from typing import List, Any

import matplotlib.pyplot as plt
import numpy as np

from slam_robot.utils.geometry import Point


class WorldItem:

    def get_collision(self, origin: Point, angle: float) -> List[Point]:
        raise NotImplementedError

    def draw(self, ax: Any, limit_inf_x=0, limit_sup_x=100, limit_inf_y=0, limit_sup_y=100, description=""):
        raise NotImplementedError


class Circle(WorldItem):
    def __init__(self, center: Point, radius: float):
        self.center = center
        self.radius = radius

    def get_collision(self, origin: Point, angle: float):
        """
        The circle is defined by its center and its radius.
        The straight line (the observation) is given by its origin and an angle.
        The intersection, if it exists, is a point that is a part of the circle and also of the straight line.

        >>> fix, ax = plt.subplots()
        >>> center = Point(10, 20)
        >>> radius = 5
        >>> circle = Circle(center, radius)
        >>> circle.draw(ax)
        >>> point = Point(0, 0)
        >>> angle = math.pi / 3
        >>> line_1 = LineByPointAndAngle(point, angle)
        >>> line_1.line_by_two_points.draw(ax)
        >>> intersection_1, intersection_2 = circle.get_collision(point, angle)
        >>> intersection_1

        >>> intersection_2

        >>> intersection_1.distance(point)

        >>> intersection_2.distance(point)

        >>> plt.xlabel("X")
        >>> plt.ylabel("Y")
        >>> plt.title("Plot")
        >>> plt.show()


        :param origin:
        :param angle:
        :return:
        """
        line = LineByPointAndAngle(origin, angle).line_by_two_points
        cartesian_line = line.cartesian_line

        distance = (math.fabs(cartesian_line.a * self.center.x + cartesian_line.b * self.center.y + cartesian_line.c)
                    / math.sqrt(cartesian_line.a ** 2 + cartesian_line.b ** 2))
        if distance > self.radius:
            return []
        elif np.isclose(distance, self.radius):
            # TODO compute the point coordinates
            raise ValueError("Very unlikely")
        # Translate the center on the origin
        centered_point_1 = line.point_1 - self.center
        centered_point_2 = line.point_2 - self.center

        d_x = centered_point_2.x - centered_point_1.x
        d_y = centered_point_2.y - centered_point_1.y
        # Pre-compute variables common to x and y equations.
        d_r_squared = d_x ** 2 + d_y ** 2
        determinant = centered_point_1.x * centered_point_2.y - centered_point_2.x * centered_point_1.y
        discriminant = self.radius ** 2 * d_r_squared - determinant ** 2

        if discriminant < 0:
            raise ValueError("The line does not intersect the circle.")

        root = math.sqrt(discriminant)

        mp = np.array([-1, 1])  # Array to compute minus/plus.
        sign = -1 if d_y < 0 else 1

        coords_x = (determinant * d_y + mp * sign * d_x * root) / d_r_squared
        coords_y = (-determinant * d_x + mp * abs(d_y) * root) / d_r_squared

        point_translated_a = Point(coords_x[0], coords_y[0])
        point_translated_b = Point(coords_x[1], coords_y[1])

        # Translate the intersection points back from origin circle to real circle.
        point_a = point_translated_a + self.center
        point_b = point_translated_b + self.center

        return [point_a, point_b]

    def draw(self, ax, limit_inf_x=0, limit_sup_x=100, limit_inf_y=0, limit_sup_y=100, description=""):
        circle = plt.Circle(self.center.to_tuple(), self.radius, edgecolor="green", facecolor="none")
        ax.add_patch(circle)


class CartesianLine(WorldItem):
    def __init__(self, a, b, c):
        """
        Line defined by equation a * x + b * y + c = 0
        :param a:
        :param b:
        :param c:
        """
        self.a = a
        self.b = b
        self.c = c

    def get_collision(self, origin: Point, angle: float):
        """

        >>> fix, ax = plt.subplots()
        >>> center = Point(10, 20)
        >>> radius = 5
        >>> circle = Circle(center, radius)
        >>> circle.draw(ax)
        >>> point = Point(0, 0)
        >>> angle = math.pi / 3
        >>> line_1 = LineByPointAndAngle(point, angle)
        >>> line_1.line_by_two_points.draw(ax)
        >>> intersection_1, intersection_2 = circle.get_collision(point, angle)
        >>> intersection_1

        >>> intersection_2

        >>> intersection_1.distance(point)

        >>> intersection_2.distance(point)

        >>> plt.xlabel("X")
        >>> plt.ylabel("Y")
        >>> plt.title("Plot")
        >>> plt.show()


        :param origin:
        :param angle:
        :return: None if no intersection, Point otherwise.
        """

        # a_origin = origin.y - next_to_origin.y
        # b_origin = next_to_origin.x - origin.x
        # c_origin = next_to_origin.y * origin.x - next_to_origin.x * origin.y
        # return self._get_intersection_with_other(CartesianLine(a_origin, b_origin, c_origin))
        return self._get_intersection_with_other(LineByPointAndAngle(origin, angle).line_by_two_points.cartesian_line)

    def _get_intersection_with_other(self, other) -> List[Point]:
        if np.isclose(np.abs(self.a * other.b - other.a * self.b), 0):
            return []
        return [Point((self.c * other.b - other.c * self.b)
                      / (self.a * other.b - other.a * self.b),
                      (self.a * other.c - other.a * self.c)
                      / (self.a * other.b - other.a * self.b))]

    def from_x(self, x):
        if self.b != 0:
            return (self.c - self.a * x) / self.b
        return x

    def draw(self, ax: Any, limit_inf_x=0, limit_sup_x=100, limit_inf_y=0, limit_sup_y=100, description=""):
        if self.b != 0:
            y_inf = self.from_x(limit_inf_x)
            y_sup = self.from_x(limit_sup_y)
            ax.plot([limit_inf_x, limit_sup_x], [y_inf, y_sup], color="black", linestyle="--")
        else:
            x = self.c / self.a
            ax.plot([x, x], [limit_inf_y, limit_sup_y], color="black", linestyle="--")


class LineByTwoPoints(WorldItem):
    def __init__(self, point_1: Point, point_2: Point):
        self.point_1 = point_1
        self.point_2 = point_2

    @property
    def cartesian_line(self) -> CartesianLine:
        a = self.point_2.y - self.point_1.y
        b = self.point_1.x - self.point_2.x
        c = -self.point_1.y * self.point_2.x + self.point_1.x * self.point_2.y
        return CartesianLine(a, b, c)

    def get_collision(self, origin: Point, angle: float) -> List[Point]:
        """
        None if no collision.
        Otherwise, returns the collision position.

        >>> fix, ax = plt.subplots()
        >>> line_2 = LineByTwoPoints(Point(10, 20), Point(20, 10))
        >>> line_2.draw(ax)

        >>> point = Point(0, 0)
        >>> angle = math.pi / 3
        >>> line_1 = LineByPointAndAngle(point, angle)
        >>> line_1.line_by_two_points.draw(ax)
        >>> intersection_1 = line_2.get_collision(point, angle)[0]
        >>> intersection_1

        >>> intersection_1.distance(point)

        >>> plt.xlabel("X")
        >>> plt.ylabel("Y")
        >>> plt.title("Plot")
        >>> plt.show()


        :param origin:
        :param angle:
        :return:
        """
        return self.cartesian_line.get_collision(origin, angle)
        # return np.abs(a*origin.x+b*origin.y+c_origin)/np.sqrt(a**2+b**2)

    def draw(self, ax: Any, limit_inf_x=0, limit_sup_x=100, limit_inf_y=0, limit_sup_y=100, description=""):
        self.cartesian_line.draw(ax, limit_inf_x, limit_sup_x, limit_inf_y, limit_sup_y)


class LineByPointAndAngle:
    def __init__(self, point: Point, angle: float):
        self.point = point
        self.angle = angle

    @property
    def next_point(self):
        return Point(self.point.x + math.cos(self.angle), self.point.y + math.sin(self.angle))
        # normalised_angle = self.angle % np.pi * 2
        # if np.isclose(normalised_angle, np.pi / 2):
        #     next_to_origin = Point(self.point.x, self.point.y + 1)
        # elif np.isclose(normalised_angle, 3 * np.pi / 2):
        #     next_to_origin = Point(self.point.x, self.point.y - 1)
        # else:
        #     sign = np.pi
        #     next_to_origin = Point(self.point.x + 1, self.point.y + math.tan(normalised_angle))
        # return next_to_origin

    @property
    def line_by_two_points(self) -> LineByTwoPoints:
        return LineByTwoPoints(self.point, self.next_point)

    def is_in_same_sense(self, other: Point) -> bool:
        """
        >>> point = Point(2, 3)
        >>> line = LineByPointAndAngle(point, 0)
        >>> line.is_in_same_sense(Point(1, 3))

        >>> line.is_in_same_sense(Point(3, 3))

        :param other:
        :return:
        """
        good_direction = self.next_point - self.point
        collision_direction = other - self.point
        return good_direction.x*collision_direction.x + good_direction.y*collision_direction.y > 0




class Form(WorldItem):
    def __init__(self, *args: List[Point]):
        self.points = args

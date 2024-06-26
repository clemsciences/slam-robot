#!/home/pi/lidar-processor/lidar_env/bin/python

"""
Geometry module.
"""
import math
from typing import Union

__author__ = ["Clément Besnier", ]

import numpy as np


class Point:
    def __init__(self, x: float, y: float):
        self.x = x
        self.y = y

    def __add__(self, other):
        return Point(self.x + other.x, self.y + other.y)

    def __sub__(self, other):
        return Point(self.x-other.x, self.y-other.y)

    def multiply_by(self, a):
        return Point(a * self.x, a * self.y)

    def distance(self, other):
        return np.sqrt(np.power(self.x - other.x, 2) + np.power(self.y - other.y, 2))

    def __str__(self):
        return "(" + str(self.x) + "," + str(self.y) + ")"

    def __repr__(self):
        return f"Point({self.x}, {self.y})"

    def rotate(self, angle: float):
        """
        >>> point = Point(2., 3.)
        >>> point.rotate(np.pi)
        >>> np.isclose(point.x, -2)
        True
        >>> np.isclose(point.y, -3)
        True

        :param angle:
        :return:
        """
        res = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]]) @ np.array([self.x, self.y]).T
        self.x = res[0]
        self.y = res[1]
        return Point(self.x, self.y)

    def copy(self):
        return Point(self.x, self.y)

    # region array
    def to_array(self) -> np.ndarray:
        return np.array([self.x, self.y])

    @staticmethod
    def from_array(point: Union[np.ndarray]):
        return Point(point[0], point[1])
    # endregion

    # region tuple
    def to_tuple(self):
        return self.x, self.y

    @classmethod
    def from_tuple(cls, point_tuple):
        assert len(point_tuple) == 2
        return cls(point_tuple[0], point_tuple[1])

    # endregion

    # region to vector
    def from_other_point_to_vector(self, other_point):
        vector = Vector()
        vector.set_by_point(other_point - self)
        return vector

    def from_angle_to_vector(self, angle: float):
        vector = Vector()
        vector.set_coordinates(1, math.tan(angle))
        return vector
    # endregion

    # region polar
    @classmethod
    def from_polar(cls, angle, distance):
        return cls(distance*np.cos(angle), distance*np.sin(angle))

    def to_angle(self):
        if self.x != 0:
            if self.x > 0 and self.y >= 0:
                return math.atan(self.y / self.x)
            elif self.x > 0 > self.y:
                return math.atan(self.y / self.x) + 2 * math.pi
            elif self.x < 0:
                return math.atan(self.y / self.x) + math.pi
            elif self.x == 0 and self.y > 0:
                return math.pi / 2
            elif self.x == 0 and self.y < 0:
                return 3 * math.pi / 2
            else:
                return 0
        else:
            return math.pi / 2

    def to_distance(self):
        return np.sqrt(self.x**2 + self.y**2)
    # endregion


class Vector:
    def __init__(self):
        self.x = 0
        self.y = 0

    def set_by_points(self, point1, point2):
        self.x = point2.x - point1.x
        self.y = point2.y - point1.y

    def set_by_point(self, point):
        self.x = point.x
        self.y = point.y

    def create_unitary(self, angle):
        self.x = np.cos(angle)
        self.y = np.sin(angle)

    def scalar_product(self, other):
        return self.x*other.x+self.y*other.y

    def compute_angle(self):
        return np.arctan(self.y / self.x)

    def compute_distance(self):
        return np.sqrt(self.x ** 2 + self.y ** 2)

    def apply_to_point(self, point: Point):
        # print(point)
        return Point(point.x + self.x, point.y + self.y)

    def __str__(self):
        return str(self.x) + " - " + str(self.y)

    def set_coordinates(self, x: float, y: float):
        self.x = x
        self.y = y

    def rotate(self, angle: float):
        """
        >>> v = Vector()
        >>> v.set_coordinates(2., 3.)
        >>> v.rotate(np.pi)
        >>> np.isclose(v.x, -2)
        True
        >>> np.isclose(v.y, -3)
        True

        :param angle:
        :return:
        """
        res = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]]) @ np.array([self.x, self.y]).T
        self.x = res[0]
        self.y = res[1]
        new_vec = Vector()
        new_vec.set_coordinates(self.x, self.y)
        return new_vec

    def __add__(self, other):
        if isinstance(other, Vector):
            v = Vector()
            v.set_coordinates(self.x+other.x, self.y+other.y)
            return v

    def __sub__(self, other):
        if isinstance(other, Vector):
            v = Vector()
            v.set_coordinates(self.x-other.x, self.y-other.y)
            return v

    def __mul__(self, other):
        if type(other) in [int, float]:
            v = Vector()
            v.set_coordinates(self.x*other, self.y*other)
            return v

    def __rmul__(self, other):
        if type(other) in [int, float]:
            v = Vector()
            v.set_coordinates(self.x*other, self.y*other)
            return v

    def multiplicate_by(self, a):
        self.x *= a
        self.y *= a

    def compute_basis_angle(self):
        """
        Computes angle to the (Ox) axis.
        :return:
        """
        vect_basis = Vector()
        vect_basis.set_coordinates(1, 0)

        return np.arccos(self.scalar_product(vect_basis)/(self.compute_distance()))

    def copy(self):
        v = Vector()
        v.set_coordinates(self.x, self.y)
        return v

    @staticmethod
    def from_array(a: np.ndarray):
        v = Vector()
        v.set_coordinates(a[0], a[1])
        return v


class Segment:
    def __init__(self, p1: Point, p2: Point):
        self.p1 = p1
        self.p2 = p2
        self.x_difference = self.compute_x_difference()
        self.y_difference = self.compute_y_difference()
        # self.determinant = self.compute_determinant()

    def __str__(self):
        return "p1: "+str(self.p1)+", p2: "+str(self.p2)

    # def compute_determinant(self):
    #     return self.p1.x * self.p2.y - self.p1.y * self.p2.x

    def compute_x_difference(self):
        return self.p2.x - self.p1.x

    def compute_y_difference(self):
        return self.p2.y - self.p1.y

    # def get_determinant(self):
    #     return self.determinant

    def get_x_difference(self):
        return self.x_difference

    def get_y_difference(self):
        return self.y_difference

    def collide(self, other):
        """
        if denominator are of same sign and numerator is lesser than denominator, then the segments collide

        >>> pa = Point(1.8, 2.1)
        >>> pb = Point(0.8, 1.1)
        >>> pc = Point(1, 1.25)
        >>> pd = Point(0, 1.25)
        >>> s1 = Segment(pa, pb)
        >>> print(s1)

        >>> s2 = Segment(pc, pd)
        >>> print(s2)

        >>> s1.collide(s2)

        >>> s2.collide(s1)


        >>> pa = Point(-1., 0.5)
        >>> pb = Point(1., 0.5)
        >>> pc = Point(0., 1.)
        >>> pd = Point(0., 2.)
        >>> s1 = Segment(pa, pb)
        >>> print(s1)

        >>> s2 = Segment(pc, pd)
        >>> print(s2)

        >>> s1.collide(s2)

        >>> s2.collide(s1)

        :param other:
        :return:
        """
        false_segment = Segment(self.p1, other.p1)

        numerator = (false_segment.get_x_difference() * other.get_y_difference() -
                     false_segment.get_y_difference() * other.get_x_difference())

        denominator = (self.get_x_difference() * other.get_y_difference() -
                       self.get_y_difference() * other.get_x_difference())
        t = numerator / denominator
        # print("t", t)
        if 0 <= t <= 1:
            print("x: ", self.p1.x + t * (self.p2.x - self.p1.x))
            print("y: ", self.p1.y + t * (self.p2.y - self.p1.y))

        # px = (self.get_determinant() * other.get_x_difference() - self.get_x_difference() * other.get_determinant())
        # / \ denominator
        # py = (self.get_determinant() * other.get_y_difference() - self.get_y_difference() * other.get_determinant())
        # / \ denominator
        return ((0 <= denominator and 0 <= numerator) or (denominator <= 0 and numerator <= 0)) and \
               (abs(numerator) <= abs(denominator))

        # if 0 < denominator and 0 < numerator or denominator < 0 and numerator < 0:
        #     return numerator < denominator
        # else:
        #     return False


def from_lidar_to_table(point: Point, robot_position: Point, robot_orientation: float) -> Point:
    """

    :param point: cartesian coordinates
    :param robot_position: cartesian coordinates
    :param robot_orientation:
    :return:
    """
    return point.rotate(robot_orientation) + robot_position


def from_theoretical_table_to_lidar(point: Point, robot_position: Point, robot_orientation: float) -> Point:
    """

    :param point:
    :param robot_position:
    :param robot_orientation:
    :return:
    """
    return (point - robot_position).rotate(np.pi/2 - robot_orientation)


def from_real_table_to_lidar(point: Point, robot_position: Point, robot_orientation: float) -> Point:
    """

    :param point:
    :param robot_position:
    :param robot_orientation:
    :return:
    """
    return (point - robot_position).rotate(robot_orientation)


def from_measured_and_expected_beacon_position_to_actual_robot_position(measured_position: Point,
                                                                        expected_position: Point):
    """

    :param measured_position:
    :param expected_position:
    :return:
    """
    angle = np.arctan(measured_position.y / measured_position.x)
    robot_position = expected_position - measured_position.rotate(angle)
    return robot_position.x, robot_position.y, angle


# def from_table_to_lidar(point: Point, robot_position: Point, robot_orientation: float) -> Point:
#     return (point - robot_position).rotate(np.pi/2 - robot_orientation)

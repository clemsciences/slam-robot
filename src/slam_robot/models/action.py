""""""
import math
from abc import abstractmethod
from typing import Optional

from slam_robot.utils.geometry import Point


class Action:
    ACTION_NAME = ""

    @property
    @abstractmethod
    def duration(self) -> float:
        pass

    def apply(self, robot, world) -> float:
        raise NotImplementedError

    def get_state(self, robot, world, timestamp, initial_position: Point, initial_orientation: float):
        raise NotImplementedError


class Move(Action):
    ACTION_NAME = "Move"

    def __init__(self, duration: Optional[float], velocity: Optional[float], distance: Optional[float]=None):
        assert (duration is not None and distance is None) or (duration is None and distance is not None)
        self._duration = distance / velocity
        self.velocity = velocity
        self.distance = distance

    def apply(self, robot, world):
        if self.velocity:
            robot.set_velocity(self.velocity)
        if self.duration:
            robot.move(self.duration)
        elif self.distance is not None:
            self._duration = self.distance/self.velocity
            robot.move(self.duration)
        return self.duration

    @classmethod
    def from_objective(cls, velocity: Optional[float], distance: float):
        return cls(None, velocity, distance)

    @property
    def duration(self):
        return self._duration

    def get_state(self, robot, world, timestamp, initial_position, initial_orientation):
        if not self.duration:
            self._duration = self.distance / self.velocity
        if timestamp >= self.duration:
            new_position = initial_position + Point(self.velocity * self.duration * math.cos(initial_orientation),
                                                    self.velocity * self.duration * math.sin(initial_orientation))
            return new_position, initial_orientation
        else:
            new_position = initial_position + Point(self.velocity * timestamp * math.cos(initial_orientation),
                                                    self.velocity * timestamp * math.sin(initial_orientation))
            return new_position, initial_orientation


class Turn(Action):
    ACTION_NAME = "Turn"

    def __init__(self, duration: Optional[float], rotation_velocity: Optional[float], angle: Optional[float]):
        assert (duration is not None and angle is None) or (duration is None and angle is not None)
        self._duration = angle / rotation_velocity
        self.rotation_velocity = rotation_velocity
        self.angle = angle

    def apply(self, robot, world):
        if self.rotation_velocity is not None:
            robot.set_rotation_velocity(self.rotation_velocity)
        if self.duration:
            robot.turn(self.duration)
        elif self.angle is not None:
            self._duration = self.angle/self.rotation_velocity
            robot.turn(self.duration)
        return self.duration

    @classmethod
    def from_objective(cls, rotation_velocity: Optional[float], angle: float):
        return cls(None, rotation_velocity, angle)

    @property
    def duration(self):
        return self._duration

    def get_state(self, robot, world, timestamp, initial_position, initial_orientation):
        if not self.duration:
            self._duration = self.angle / self.rotation_velocity
        if timestamp >= self.duration:
            return initial_position, initial_orientation + self.rotation_velocity * self.duration
        else:
            return initial_position, initial_orientation + self.rotation_velocity * timestamp


class Wait(Action):
    ACTION_NAME = "Wait"

    def __init__(self, duration: Optional[float]):
        self._duration = duration

    def apply(self, robot, world):
        return self.duration

    @property
    def duration(self):
        return self._duration

    def get_state(self, robot, world, timestamp, initial_position, initial_orientation):
        return initial_position, initial_orientation


class Sense(Action):
    ACTION_NAME = "Sense"

    def __init__(self):
        self._duration = 0

    def apply(self, robot, world):
        robot.sense(world)
        return 0

    @property
    def duration(self):
        return self._duration

    def get_state(self, robot, world, timestamp, initial_position, initial_orientation):
        return initial_position, initial_orientation


class SetVelocity(Action):
    ACTION_NAME = "SetVelocity"
    def __init__(self, velocity: float):
        self._duration = 0
        self.velocity = velocity

    def apply(self, robot, world):
        robot.set_velocity(self.velocity)
        return self.duration

    @property
    def duration(self):
        return self._duration

    def get_state(self, robot, world, timestamp, initial_position, initial_orientation):
        return initial_position, initial_orientation


class SetRotationVelocity(Action):
    ACTION_NAME = "SetRotationVelocity"

    def __init__(self, rotation_velocity: float):
        self._duration = 0
        self.rotation_velocity = rotation_velocity

    def apply(self, robot, world):
        robot.set_rotation_velocity(self.rotation_velocity)
        return self.duration

    @property
    def duration(self):
        return self._duration

    def get_state(self, robot, world, timestamp, initial_position, initial_orientation):
        return initial_position, initial_orientation

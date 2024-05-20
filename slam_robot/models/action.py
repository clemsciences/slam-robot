""""""
from typing import Optional


class Action:

    def apply(self, robot, world) -> float:
        raise NotImplementedError


class Move(Action):
    def __init__(self, duration: Optional[float], velocity: Optional[float], distance: Optional[float]=None):
        assert (duration is not None and distance is None) or (duration is None and distance is not None)
        self.duration = duration
        self.velocity = velocity
        self.distance = distance

    def apply(self, robot, world):
        if self.velocity:
            robot.set_velocity(self.velocity)
        if self.duration:
            robot.move(self.duration)
        elif self.distance is not None:
            self.duration = self.distance/self.velocity
            robot.move(self.duration)
        return self.duration

    @classmethod
    def from_objective(cls, velocity: Optional[float], distance: float):
        return cls(None, velocity, distance)


class Turn(Action):
    def __init__(self, duration: Optional[float], rotation_velocity: Optional[float], angle: Optional[float]):
        assert (duration is not None and angle is None) or (duration is None and angle is not None)
        self.duration = duration
        self.rotation_velocity = rotation_velocity
        self.angle = angle

    def apply(self, robot, world):
        if self.rotation_velocity is not None:
            robot.set_rotation_velocity(self.rotation_velocity)
        if self.duration:
            robot.turn(self.duration)
        elif self.angle is not None:
            self.duration = self.angle/self.rotation_velocity
            robot.turn(self.duration)
        return self.duration

    @classmethod
    def from_objective(cls, rotation_velocity: Optional[float], angle: float):
        return cls(None, rotation_velocity, angle)


class Wait(Action):
    def __init__(self, duration: Optional[float]):
        self.duration = duration

    def apply(self, robot, world):
        return self.duration


class Sense(Action):
    def __init__(self):
        self.duration = 0

    def apply(self, robot, world):
        robot.sense(world)
        return 0


import math
from typing import Any, List, Optional

import numpy as np

from slam_robot.models.action import Action
from slam_robot.models.perception import RobotPerception
from slam_robot.models.world import World
from slam_robot.utils.geometry import Point


class Robot:
    def __init__(self, initial_position: Point, initial_orientation: float):
        # region robot state
        self.position = initial_position
        self.orientation = initial_orientation
        self.velocity = 0.1
        self.rotation_velocity = 0.1
        # endregion

        # region sensor
        self.angle_measures = 300
        self.measure_max_distance = 1000
        # endregion

        # region uncertainty
        self.rotation_noise = 0.01
        self.translation_noise = 0.01
        # self.velocity_noise = 0.01

        # endregion

        # self.world_knowledge = []
        self.actions: List[Action] = []
        self.measures: List[RobotPerception] = []
        self.lifetime = 0

    # region actions
    def move(self, duration: float) -> Point:
        # TODO add translation_noise
        translation = Point(self.velocity * duration * math.cos(self.orientation), self.velocity * duration * math.sin(self.orientation))
        self.position += translation
        return self.position

    def turn(self, duration: float) -> float:
        # TODO add rotation_noise
        self.orientation += self.rotation_velocity * duration
        return self.orientation

    def apply_action(self, action: Action, world):
        self.lifetime += action.apply(self, world)
        self.actions.append(action)

    def apply_actions(self, actions: List[Action], world):
        for action in actions:
            print(action)
            action.apply(self, world)

    def set_velocity(self, velocity: float):
        self.velocity = velocity

    def set_rotation_velocity(self, velocity: float):
        self.rotation_velocity = velocity

    # endregion

    def sense(self, world: World) -> List[Optional[Point]]:
        obstacles = []
        for angle in np.linspace(0, 2 * np.pi, self.angle_measures):
            obstacle = world.see_obstacles(self.position, angle)
            if obstacle and obstacle.distance(self.position) < self.measure_max_distance:
                obstacles.append(obstacle)
        self.add_measure(obstacles)
        return obstacles

    def draw(self, ax: Any):
        ax.scatter([self.position.x], [self.position.y], color='red', marker='o', s=20)

    def add_measure(self, obstacles: List[Point]):
        self.measures.append(RobotPerception(self.lifetime, obstacles, self.position))



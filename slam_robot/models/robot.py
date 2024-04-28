import math
from typing import Any

import numpy as np

from slam_robot.models.world import World
from slam_robot.utils.geometry import Point


class Robot:
    def __init__(self, initial_position: Point, initial_orientation: float):
        self.position = initial_position
        self.orientation = initial_orientation
        self.velocity = 0

        self.angle_measures = 60
        self.measure_max_distance = 1000

        self.world_knowledge = []

    def sense(self, world: World):
        obstacles = []
        for angle in np.linspace(0, 2 * np.pi, self.angle_measures):
            obstacle = world.see_obstacles(self.position, angle)
            if obstacle and obstacle.distance(self.position) < self.measure_max_distance:
                obstacles.append(obstacle)


































































        return obstacles

    def move(self, distance: float):
        self.position += Point(self.velocity * distance * math.cos(self.orientation),
                               self.velocity * distance * math.sin(self.orientation))

    def turn(self, orientation: float):
        self.orientation = orientation

    def draw(self, ax: Any):
        ax.scatter([self.position.x], [self.position.y])



"""

"""
from typing import Optional, List, Any

import numpy as np

from slam_robot.utils.geometry import Point


__author__ = ["Clément Besnier"]


class Trajectory:

    def __init__(self,
                 positions: Optional[List[Point]] = None,
                 orientations: Optional[List[float]] = None,
                 timestamps: Optional[List[float]] = None):
        if positions is not None:
            self.positions = positions
        else:
            self.positions = []
        if orientations is not None:
            self.orientations = orientations
        else:
            self.orientations = []
        if timestamps is not None:
            self.timestamps = timestamps
        else:
            self.timestamps = []

    def __len__(self):
        return len(self.positions)

    def __getitem__(self, item: int):
        if 0 <= item < len(self.positions):
            return RobotSnapshot(self.positions[item], self.orientations[item], self.timestamps[item])
        raise IndexError

    def add_position_noise(self, scale: float):
        positions = []
        for position in self.positions:
            noisy_position = position + Point(np.random.normal(0, scale), np.random.normal(0, scale))
            positions.append(noisy_position)
        self.positions = positions

    def add_orientation_noise(self, scale: float):
        orientations = []
        for orientation in self.orientations:
            noisy_orientation = orientation + np.random.normal(0, scale)
            orientations.append(noisy_orientation)
        self.orientations = orientations

    def draw(self, ax: Any):
        for position_1, position_2 in zip([self.positions[:-1]], self.positions[1:]):
            ax.axline(position_1.x, position_1.y, position_2.x, position_2.y)
            # ax.scatter([position.x], [position.y], color='red', marker='o', s=20)


class RobotSnapshot:
    def __init__(self, position: Point, orientation: float, timestamp: float):
        self.position = position
        self.orientation = orientation
        self.timestamp = timestamp


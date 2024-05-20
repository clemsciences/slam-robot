from typing import List, Optional

from slam_robot.utils.geometry import Point


class RobotPerception:
    def __init__(self,
                 timestamp: float,
                 obstacles: List[Optional[Point]],
                 position: Point,
                 ):
        self.timestamp = timestamp
        self.obstacles = obstacles
        self.position = position

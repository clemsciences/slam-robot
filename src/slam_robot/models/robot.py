import math
from typing import Any, List, Optional

import numpy as np

from slam_robot.models.action import Action
from slam_robot.models.perception import RobotPerception
from slam_robot.models.trajectory import Trajectory
from slam_robot.models.world import World
from slam_robot.utils.geometry import Point


class Robot:
    def __init__(self, initial_position: Point, initial_orientation: float):
        # region robot state
        self.initial_position = initial_position
        self.position = initial_position
        self.initial_orientation = initial_orientation
        self.orientation = initial_orientation
        self.initial_velocity = 0.1
        self.velocity = 0.1
        self.initial_rotation_velocity = 0.1
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

    def draw(self, ax: Any, with_orientation: bool):
        ax.scatter([self.position.x], [self.position.y], color='red', marker='o', s=20)
        ax.set_aspect('equal', adjustable='datalim')
        if with_orientation:
            vector = self.position.from_angle_to_vector(self.orientation)
            other_position = vector.apply_to_point(self.position)
            ax.arrow(self.position.x, self.position.y, math.cos(self.orientation), math.sin(self.orientation),
                     head_width=0.05, head_length=0.05, fc="k", ec="k")

    def add_measure(self, obstacles: List[Point]):
        self.measures.append(RobotPerception(self.lifetime, obstacles, self.position))

    def recover_trajectory(self, actions: List[Action], world: World, time_step: float):
        # print([action.duration for action in actions])
        total_duration = sum([action.duration for action in actions])
        current_timestamp = 0
        previous_actions_duration = 0

        positions = [self.initial_position]
        orientations = [self.initial_orientation]
        timestamps = [0]

        initial_position = self.initial_position
        initial_orientation = self.initial_orientation

        action_index = 0
        # print("*****************")
        # print(actions[action_index].ACTION_NAME)
        while current_timestamp < total_duration:
            if current_timestamp >= previous_actions_duration + actions[action_index].duration:
                if action_index < len(actions) - 1:
                    new_position, new_orientation = actions[action_index].get_state(self, world,
                                                                                    actions[action_index].duration,
                                                                                    initial_position,
                                                                                    initial_orientation)
                    positions.append(new_position)
                    orientations.append(new_orientation)
                    timestamps.append(previous_actions_duration)
                    previous_actions_duration += actions[action_index].duration
                    initial_position = positions[len(positions) - 1]
                    initial_orientation = orientations[len(orientations) - 1]
                    action_index += 1
                    # print("*****************")
                    # print(actions[action_index].ACTION_NAME)
                else:
                    # print("FINSH")
                    break
            # print(previous_actions_duration)
            # print(current_timestamp)
            # print(f"action duration: {current_timestamp - previous_actions_duration}")

            new_position, new_orientation = actions[action_index].get_state(self, world,
                                                                            current_timestamp - previous_actions_duration,
                                                                            initial_position,
                                                                            initial_orientation)
            current_timestamp += time_step
            positions.append(new_position)
            orientations.append(new_orientation)
            timestamps.append(current_timestamp)
            # if actions[action_index].ACTION_NAME == "Move":
            #     print(f"new position {new_position}")
            # elif actions[action_index].ACTION_NAME == "Turn":
            #     print(f"new orientation {new_orientation}")
        return Trajectory(positions, orientations, timestamps)





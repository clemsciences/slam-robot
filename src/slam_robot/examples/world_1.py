import numpy as np

from slam_robot.models.action import Turn, Move, Sense
from slam_robot.models.robot import Robot
from slam_robot.models.world import World
from slam_robot.models.world_items import Circle, LineByTwoPoints
from slam_robot.utils.geometry import Point

import matplotlib.pyplot as plt

plt.rcParams["figure.figsize"] = (5, 5)

world_1 = World(
    [
        LineByTwoPoints(Point(0, 0), Point(0, 100)),
        LineByTwoPoints(Point(0, 100), Point(100, 100)),
        LineByTwoPoints(Point(100, 100), Point(100, 0)),
        LineByTwoPoints(Point(100, 0), Point(0, 0)),
        Circle(Point(50, 50), 10),
    ],
    100,
    100
)

robot = Robot(Point(12, 12), 0)

actions = [
    Turn.from_objective(1.5, np.pi/3),
    Sense(),
    Move.from_objective(10, 10),
    Sense(),
    Turn.from_objective(1.5, np.pi/6),
    Move.from_objective(10, 30),
    Sense(),
    Move.from_objective(10, 30),
    Sense()
]

robot.apply_actions(actions, world_1)


def show(world: World,
         robot: Robot,
         show_world: bool = True,
         show_measures: bool = True,
         show_robot: bool = True,
         show_clusters: bool = False):
    for measure in robot.measures:
        fix, ax = plt.subplots()
        if show_world:
            world.draw(ax)
        if show_measures:
            x_values = []
            y_values = []
            print(measure.position)
            for point in measure.obstacles:
                line = LineByTwoPoints(measure.position, point)
                # line.draw(ax)
                # x_values.append(point.x + robot.position.x)
                x_values.append(point.x)
                # y_values.append(point.y + robot.position.y)
                y_values.append(point.y)
            plt.scatter(x_values, y_values, color="blue")
        if show_clusters:
            clusters = measure.clusterize()
            for cluster in clusters:
                plt.scatter(cluster.x_points, cluster.y_points)
        if show_robot:
            # robot.draw(ax)
            plt.scatter([measure.position.x], [measure.position.y], color="red")

        plt.xlabel("X")
        plt.ylabel("Y")
        plt.title("Plot")
        plt.show()


# show(world_1, robot, show_world=True, show_measures=True, show_robot=True)
show(world_1, robot, show_world=True, show_measures=False, show_robot=False, show_clusters=True)

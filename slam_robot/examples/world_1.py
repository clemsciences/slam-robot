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

for measure in robot.measures:
    fix, ax = plt.subplots()

    world_1.draw(ax)

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
    # print([(x_values[i], y_values[i]) for i in range(len(x_values))])
    # robot.draw(ax)
    plt.scatter([measure.position.x], [measure.position.y], color="red")

    plt.xlabel("X")
    plt.ylabel("Y")
    plt.title("Plot")
    plt.show()




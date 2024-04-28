import numpy as np

from slam_robot.models.robot import Robot
from slam_robot.models.world import World
from slam_robot.models.world_items import Circle, LineByTwoPoints
from slam_robot.utils.geometry import Point

import matplotlib.pyplot as plt


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

what_the_robot_sees = robot.sense(world_1)
print(what_the_robot_sees)


fix, ax = plt.subplots()

world_1.draw(ax)
robot.draw(ax)

x_values = []
y_values = []
for point in what_the_robot_sees:
    line = LineByTwoPoints(robot.position, point)
    line.draw(ax)
    # x_values.append(point.x + robot.position.x)
    x_values.append(point.x)
    # y_values.append(point.y + robot.position.y)
    y_values.append(point.y)
plt.scatter(x_values, y_values, color="blue")
print([(x_values[i], y_values[i]) for i in range(len(x_values))])

plt.xlabel("X")
plt.ylabel("Y")
plt.title("Plot")
plt.show()




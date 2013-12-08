import numpy as np
import matplotlib.pyplot as plt
import time

# Global variables, can be used by any process
dt = 0.1
time = 0

class Map:
  def __init__(self, dimensions, obstacles=[], robots=[]):
    self.width = dimensions[0]
    self.height = dimensions[1]
    self.obstacles = obstacles
    self.robots = robots

  def addObstacle(self, obstacle):
    self.obstacles.append(obstacle)

  def addRobot(self, robot):
    self.robots.append(robot)


class Robot:
  def __init__(self, position, velocity):
    self.position = position
    self.velocity = velocity


class Obstacle:
  def __init__(self, function):
    self.function = function



if __name__ == '__main__':



import numpy as np
import matplotlib.pyplot as plt
import time

# Global variables, can be used by any process
dt = 0.1
time = 0
goalTolerance=2.0

class Map:
  def __init__(self, dimensions, goalposition, endTime, obstacles=[], robots=[]):
    self.width = dimensions[0]#eg, max x is width, min x is 0
    self.height = dimensions[1]
    self.obstacles = obstacles
    self.robots = robots#list of robots
    self.goalPosition = goalPosition#tuple of goalx and goaly
    #self.goaltolerance = goaldimensions[1]
    self.endTime = endTime

  def addObstacle(self, obstacle):
    self.obstacles.append(obstacle)

  def addRobot(self, robot):
    self.robots.append(robot)

  def updatePosition(self,robotId):
    if self.robots[robotId].finished == True:
      return
    else:
      self.robots[robotId].position = (self.robots[robotId].position[0]+dt*self.robots[robotId].velocity[0],self.robots[robotId].position[1]+dt*self.robots[robotId].velocity[1])
      #min x
      if self.robots[robotId].position[0]<0:
        self.robots[robotId].position = (0,self.robots[robotId].position[1])
        self.robots[robotId].velocity = (0,self.robots[robotId].velocity[1])
      #max x
      if self.robots[robotId].position[0]>self.width:
        self.robots[robotId].position = (self.width,self.robots[robotId].position[1])
        self.robots[robotId].velocity = (0,self.robots[robotId].velocity[1])
      #min y
      if self.robots[robotId].position[1]<0:
        self.robots[robotId].position = (self.robots[robotId].position[0],0)
        self.robots[robotId].velocity = (self.robots[robotId].velocity[0],0)
      #max y
      if self.robots[robotId].position[0]>self.height:
        self.robots[robotId].position = (self.robots[robotId].position[0],self.height)
        self.robots[robotId].velocity = (self.robots[robotId].velocity[0],0)
      
      distfromgoal = np.sqrt((self.robots[robotId].position[0]-self.goalPosition[0])**2+(self.robots[robotId].position[1]-self.goalPosition[1])**2)
      
      if distfromgoal < goalTolerance:
        self.robots[robotId].finished = True



class Robot:
  def __init__(self, position, velocity, robotId):
    self.position = position
    self.velocity = velocity
    self.finished = False
    self.id = robotId


class Obstacle:
  def __init__(self, function):
    self.function = function


class CircleObstacle(Obstacle):
  def __init__(self, center, radius):
    def circle(position):
      return (position[0]-center[0])**2 + (position[1]-center[1])**2 <= radius**2

    self.function = circle

  
if __name__ == '__main__':

  startPos = (25,100)
  startVel = (1,0)
  
  obstacles = []
  obstacles.append(CircleObstacle((400, 100), 15))
  
  robots = []
  robots.append(Robot(startPos, startVel))

  globalMap = Map((500, 200), obstacles, robots)
  
  while time < globalMap.endTime:
    
    for robot in globalMap.robots:
      globalMap.updatePosition(robot.id)
      print robot.position




import numpy as np
import matplotlib.pyplot as plt
import time
import csv

# Global variables, can be used by any process
dt = 0.1
time = 0
goalTolerance=2.0

class Map:
  def __init__(self, dimensions, goalPosition, endTime, obstacles=[], robots=[]):
    self.width = dimensions[0]        #eg, max x is width, min x is 0
    self.height = dimensions[1]
    self.obstacles = obstacles
    self.robots = robots              #list of robots
    self.goalPosition = goalPosition  #tuple of goalx and goaly
    self.endTime = endTime

  def addObstacle(self, obstacle):
    self.obstacles.append(obstacle)

  def addRobot(self, robot):
    self.robots.append(robot)

  def updateVelocity(self, robotId):
    robot = self.robots[robotId]

    for obstacle in obstacles:
      if obstacle.contains(robot):
        obstacle.updateVelocity(robot)


  def updatePosition(self, robotId):
    if self.robots[robotId].finished == True:
      return
    else:
      self.robots[robotId].position = (self.robots[robotId].position[0]+dt*self.robots[robotId].velocity[0],self.robots[robotId].position[1]+dt*self.robots[robotId].velocity[1])
      #min x
      if self.robots[robotId].position[0] < 0:
        self.robots[robotId].position = (0,self.robots[robotId].position[1])
        self.robots[robotId].velocity = (0,self.robots[robotId].velocity[1])
      #max x
      if self.robots[robotId].position[0] > self.width:
        self.robots[robotId].position = (self.width,self.robots[robotId].position[1])
        self.robots[robotId].velocity = (0,self.robots[robotId].velocity[1])
      #min y
      if self.robots[robotId].position[1] < 0:
        self.robots[robotId].position = (self.robots[robotId].position[0],0)
        self.robots[robotId].velocity = (self.robots[robotId].velocity[0],0)
      #max y
      if self.robots[robotId].position[1] > self.height:
        self.robots[robotId].position = (self.robots[robotId].position[0],self.height)
        self.robots[robotId].velocity = (self.robots[robotId].velocity[0],0)
      
      distFromGoal = np.sqrt((self.robots[robotId].position[0]-self.goalPosition[0])**2+(self.robots[robotId].position[1]-self.goalPosition[1])**2)
      
      if distFromGoal < goalTolerance:
        self.robots[robotId].finished = True



class Robot:
  def __init__(self, position, velocity, robotId):
    self.position = position
    self.velocity = velocity
    self.finished = False
    self.id = robotId
    self.posHistory = [position]
    self.velHistory = [velocity]

class Obstacle:
  def __init__(self, function):
    self.contains = function


class CircleObstacle(Obstacle):
  def __init__(self, center, radius):
    def circle(robot):
      position = robot.position
      return (position[0]-center[0])**2 + (position[1]-center[1])**2 <= radius**2

    def moderateSlow(robot):
      return slowDown(0.01, robot)

    self.contains = circle
    self.updateVelocity = moderateSlow

def slowDown(slowfactor, robot, timespan=1):
  velocity = robot.velocity
  robot.velocity = (velocity[0]*(1-slowfactor) ** (dt / timespan), velocity[1]*(1-slowfactor) ** (dt / timespan))


if __name__ == '__main__':

  startPos = (25,100)
  startVel = (1,0)
  
  obstacles = []
  obstacles.append(CircleObstacle((400, 100), 15))
  
  robots = []
  robots.append(Robot(startPos, startVel, 0))

  globalMap = Map((500, 200), (450,100), 555, obstacles, robots)
  
  while time < globalMap.endTime:
    
    for robot in globalMap.robots:
      globalMap.updatePosition(robot.id)
      globalMap.updateVelocity(robot.id)
      robot.posHistory.append(robot.position)
      robot.velHistory.append(robot.velocity)

    time += dt

  x = []
  y = []

  for robot in globalMap.robots:
    for pos in robot.posHistory:
      x.append(pos[0])
      y.append(pos[1])

  plt.plot(x,y)
    
  '''
  with open('out.csv', 'wb') as csvfile:
    writer = csv.writer(csvfile)
  '''




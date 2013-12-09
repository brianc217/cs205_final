import numpy as np
import matplotlib.pyplot as plt
import time
import csv
import math

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
  def __init__(self, position, velocity, robotId, slowFactor=0.05):
    self.position = position
    self.velocity = velocity
    self.finished = False
    self.id = robotId
    self.posHistory = [position]
    self.velHistory = [velocity]
    self.V0 = math.sqrt(velocity[0]**2 + velocity[1]**2)
    self.slowFactor = slowFactor

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

class GravityObstacle(Obstacle):
  #def __init__(self, center, radius):
  def __init__(self, goalPosition):

    def always(robot):
      return True

    def gravityVelocity(robot):
      asymptoticSpeed = robot.V0
      slowfactor = robot.slowFactor
      timespan = 1
      inVelocity = robot.velocity
      inSpeed = np.sqrt(inVelocity[0]**2+inVelocity[1]**2)
      outSpeed = asymptoticSpeed-(asymptoticSpeed-inSpeed)*(1-slowfactor)**(dt/timespan)


      angleRobot = np.arctan2(robot.velocity[1],robot.velocity[0])*180./np.pi
      angleGoal = np.arctan2(goalPosition[1]-robot.position[1],goalPosition[0]-robot.position[0])*180./np.pi
      
      # angleRate = 30
      # angleDiff = (angleGoal - angleRobot-180)% 360 - 180
      # print "angleDiff",angleDiff
      # angleMotion = angleRate * (1-(1-slowfactor)**(dt/timespan))#this line should be changed if used, no slowfactor or anything
      # if np.abs(angleMotion)>np.abs(angleDiff):
      #   angleNew = angleGoal
      # else:
      #   angleNew = angleRobot+


      if angleGoal-angleRobot<180 and angleGoal-angleRobot>-180:
        angleNew = angleRobot+(angleGoal-angleRobot)*(1-(1-slowfactor)**(dt/timespan))
      else:
        angleDiff = (angleGoal-angleRobot+180) % 360 - 180
        angleNew = angleRobot+angleDiff*(1-(1-slowfactor)**(dt/timespan))
      robot.velocity = (outSpeed*np.cos(angleNew*np.pi/180),outSpeed*np.sin(angleNew*np.pi/180))
    
    self.updateVelocity = gravityVelocity
    self.contains = always

class RigidObstacle():
  def __init__(self, center, radius):

    def canceledVelocity(robot):
      pos = robot.position
      vel = robot.velocity
      e = 0.01

      if pos[1] > center[1]:
        newY = e + center[1] + math.sqrt(radius**2 - (pos[0]-center[0])**2)
      else:
        newY = e + center[1] - math.sqrt(radius**2 - (pos[0]-center[0])**2)

      newX = pos[0]

      robot.position = (newX,newY)
      mag0 = math.sqrt(vel[0]**2 + vel[1]**2)
      Vx0 = vel[0] / mag0
      Vy0 = vel[1] / mag0

      cancelV = (newX-center[0], newY-center[1])
      mag1 = math.sqrt(cancelV[0]**2 + cancelV[1]**2)
      Vx1 = cancelV[0] / mag1
      Vy1 = cancelV[1] / mag1

      newVx = (Vx0 + Vx1) * mag0
      newVy = (Vy0 + Vy1) * mag0

      robot.velocity = (newVx,newVy)

    self.contains = CircleObstacle(center, radius).contains
    self.updateVelocity = canceledVelocity


def slowDown(slowfactor, robot, timespan=1):
  velocity = robot.velocity
  robot.velocity = (velocity[0]*(1-slowfactor) ** (dt / timespan), velocity[1]*(1-slowfactor) ** (dt / timespan))

def allRobotsInGoal(gMap):

  for robot in gMap.robots:
    if not robot.finished:
      return False

  return True

if __name__ == '__main__':

  startPos = (25,40)
  startVel = (1,4)

  goalPosition = (450,100)
  mapDim = (500,200)
  endTime = 4000
  
  circlePosition = (400,100)
  circleRadius = 30
  rigidPos = (225,100)
  rigidRad = 50

  obstacles = []
  obstacles.append(CircleObstacle(circlePosition, circleRadius))
  obstacles.append(GravityObstacle(goalPosition))
  obstacles.append(RigidObstacle(rigidPos, rigidRad))
  
  robots = []
  robots.append(Robot(startPos, startVel, 0))

  globalMap = Map(mapDim, goalPosition, endTime, obstacles, robots)

  
  while time < globalMap.endTime and not allRobotsInGoal(globalMap):
    
    for robot in globalMap.robots:

      if robot.finished == False:
        globalMap.updatePosition(robot.id)
        globalMap.updateVelocity(robot.id)
        robot.posHistory.append(robot.position)
        robot.velHistory.append(robot.velocity)
        print robot.position, robot.velocity
      else:
        print "robot",robot.id,"is in goal"

    time += dt

  x = []
  y = []

  for robot in globalMap.robots:
    for pos in robot.posHistory:
      x.append(pos[0])
      y.append(pos[1])

  plt.figure()
  plt.plot(x,y,'b-',linewidth=4)
  ax1 = plt.gca()
  plt.axis('scaled')
  ax1.set_xlim([0,mapDim[0]])
  ax1.set_ylim([0,mapDim[1]])
  dotGap = int(np.round(np.float(len(x))/100.))
  xdots = x[0::dotGap]
  ydots = y[0::dotGap]
  plt.plot(xdots,ydots,'k.',markersize=3)
  goalCircle = plt.Circle(goalPosition,goalTolerance,color='g')
  ax1.add_artist(goalCircle)
  startCircle = plt.Circle(startPos,goalTolerance,color='b')
  ax1.add_artist(startCircle)
  obstacleCircle1 = plt.Circle(circlePosition,circleRadius,color='r')
  ax1.add_artist(obstacleCircle1)
  rigidCircle = plt.Circle(rigidPos,rigidRad,color='k')
  ax1.add_artist(rigidCircle)

  plt.show()
    
  '''
  with open('out.csv', 'wb') as csvfile:
    writer = csv.writer(csvfile)
  '''




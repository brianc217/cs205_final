from mpi4py import MPI
import numpy as np
import matplotlib.pyplot as plt
import time as timepackage
import math
import random

# Global variables, can be used by any process
dt = 0.1
time = 0
goalTolerance=6.0

# Map defines the physical space, including dimensions, all robots, obstacles, goal position, etc.
class Map:
  def __init__(self, dimensions, goalPosition, endTime, obstacles=[], robots={}):
    self.width = dimensions[0]        #eg, max x is width, min x is 0
    self.height = dimensions[1]
    self.obstacles = obstacles
    self.robots = robots              #list of robots
    self.goalPosition = goalPosition  #tuple of goalx and goaly
    self.endTime = endTime

  def addObstacle(self, obstacle):
    self.obstacles.append(obstacle)

  def addRobot(self, robot):
    self.robots[robot.id] = robot

  def updateVelocity(self, robotId):
    robot = self.robots[robotId]

    for obstacle in obstacles:
      if obstacle.contains(robot):
        obstacle.updateVelocity(robot)

  # Handle position and velocity changes due to robot on robot interactions
  def robotToRobot(self, robotId, neighbors):
    # "Neighbors" denotes robots that could be in the vicinity of robotId
    for robot in neighbors:
      if robot.id is robotId:
        continue

      # Treat robots like rigid obstacles, but without a bounce factor
      neighbor = RigidObstacle(robot.position, 3, bounce=False)
      if neighbor.contains(self.robots[robotId]) and not robot.finished:
        neighbor.updateVelocity(self.robots[robotId])


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
      self.robots[robotId].endTime = time


class Robot:
  def __init__(self, position, velocity, robotId, start, slowFactor=0.05):
    self.position = position
    self.velocity = velocity
    self.finished = False
    self.id = robotId
    self.posHistory = [position]
    self.velHistory = [velocity]
    self.V0 = math.sqrt(velocity[0]**2 + velocity[1]**2)
    self.slowFactor = slowFactor
    self.histx = 0
    self.histy = 0
    self.histvx = 0
    self.histvy = 0
    self.histvmag = 0
    self.histvang = 0
    self.startTime = start
    self.endTime = float('Inf')

class Obstacle:
  def __init__(self, function):
    self.contains = function

# Circular shaped obstacle that slows down robots that pass through it
class CircleObstacle(Obstacle):
  def __init__(self, center, radius):
    def circle(robot):
      position = robot.position
      return (position[0]-center[0])**2 + (position[1]-center[1])**2 <= radius**2

    def moderateSlow(robot):
      return slowDown(0.2, robot)

    self.contains = circle
    self.updateVelocity = moderateSlow

# Obstacle that is only an obstacle in that it affects how a robot moves, attracting it to a point like gravity
class GravityObstacle(Obstacle):

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
  def __init__(self, center, radius, bounce=True):

    # Function that acts as a normal force by canceling all of a robots velocity in the direction of the obstacle
    def canceledVelocity(robot):
      pos = robot.position
      vel = robot.velocity
      e = 0.01                # Offset of position from edge of obstacle

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

      if bounce:
        newVx = (Vx0 + Vx1) * mag0
        newVy = (Vy0 + Vy1) * mag0
      else:
        newVx = (Vx0 + Vx1/mag0) * mag0
        newVy = (Vy0 + Vy1/mag0) * mag0

      robot.velocity = (newVx,newVy)

    self.contains = CircleObstacle(center, radius).contains
    self.updateVelocity = canceledVelocity


def slowDown(slowfactor, robot, timespan=1):
  velocity = robot.velocity
  robot.velocity = (velocity[0]*(1-slowfactor) ** (dt / timespan), velocity[1]*(1-slowfactor) ** (dt / timespan))


# Returns true if all the robots on the map have reached the goal
def allRobotsInGoal(gMap):
  if len(gMap.robots) is 0:
    return False

  for robot in gMap.robots:
    if not robot.finished:
      return False

  return True


if __name__ == '__main__':
  comm = MPI.COMM_WORLD
  rank = comm.Get_rank()
  size = comm.Get_size()

  numRows = 2
  numCols = 2

  assert size == numRows * numCols
  
  # Initialize variables which define the map
  startTimeProgram = timepackage.time()
  startPos = (25,40)
  startGrid = 0
  vMagnitude = 3

  goalPosition = (450,100)
  mapDim = (500,200)
  gridWidth = mapDim[0]/numCols
  gridHeight = mapDim[1]/numRows
  rowNum = rank / numCols
  colNum = rank % numCols
  bufferSize = vMagnitude + 1

  endTime = 150
  maxNumRobots = 8
  
  circlePosition = (400,100)
  circleRadius = 30
  rigidPos = (225,100)
  rigidRad = 50

  obstacles = []
  obstacles.append(CircleObstacle(circlePosition, circleRadius))
  obstacles.append(GravityObstacle(goalPosition))
  obstacles.append(RigidObstacle(rigidPos, rigidRad))

  globalMap = Map(mapDim, goalPosition, endTime, obstacles)
  counter = 0
  allGridsEmpty = False

  # Run simulation while until time has expired or all robots are in the goal
  while time < globalMap.endTime and not allGridsEmpty:
    print time
    # Release a new robot every 5 iterations
    if counter % 5 is 0 and len(globalMap.robots) < maxNumRobots and rank is startGrid:
      angle = random.random() * 180
      Vx = math.sin(math.radians(angle)) * vMagnitude
      Vy = math.cos(math.radians(angle)) * vMagnitude

      rID = len(globalMap.robots)  
      globalMap.addRobot(Robot(startPos, (Vx, Vy), rID, time))

    north, south, east, west = [], [], [], []
    # Compute which robots are in buffer
    for robot in globalMap.robots.values():
      x = robot.position[0]
      y = robot.position[1]

      if x < colNum * gridWidth + bufferSize:
        west.append(robot)
        if x < colNum * gridWidth - bufferSize:
          globalMap.robots.pop(robot.id)
        
      elif x > (colNum+1) * gridWidth - bufferSize:
        east.append(robot)
        if x > (colNum+1) * gridWidth + bufferSize:
          globalMap.robots.pop(robot.id)

      if y < rowNum * gridHeight + bufferSize:
        north.append(robot)
        if y < rowNum * gridHeight - bufferSize:
          globalMap.robots.pop(robot.id)

      elif y > (rowNum+1) * gridHeight - bufferSize:
        south.append(robot)
        if y > (rowNum+1) * gridHeight + bufferSize:
          globalMap.robots.pop(robot.id)

    print north, south, east, west

    # Send left
    eastBuf = []
    for i in range(numCols-1):
      if colNum == i:
        eastBuf = comm.recv(source=rank+1)
      elif colNum == i+1:
        comm.send(obj=west, dest=rank-1)

    westBuf = []
    for i in range(numCols-1):
      if colNum == i:
        comm.send(obj=east, dest=rank+1)
      elif colNum == i+1:
        westBuf = comm.recv(source=rank-1)

    northBuf = []
    for i in range(numRows-1):
      if rowNum == i:
        comm.send(obj=south, dest=rank+numCols)
      elif rowNum == i+1:
        northBuf = comm.recv(source=rank-numCols)

    southBuf = []
    for i in range(numRows-1):
      if rowNum == i:
        southBuf = comm.recv(source=rank+numCols)
      elif rowNum == i+1:
        comm.send(obj=north, dest=rank-numCols)

    newRobots = eastBuf + westBuf + northBuf + southBuf

    for nRobot in newRobots:
      robotExists = False
      for robot in globalMap.robots.values():
        if robot.id == nRobot.id:
          robotExists = True

      if not robotExists:
        globalMap.addRobot(nRobot)   

    # Update position and velocity of the robot taking into account all obstacles and other robots
    for robot in globalMap.robots.values():

      if robot.finished == False:
        globalMap.updatePosition(robot.id)
        globalMap.updateVelocity(robot.id)
        globalMap.robotToRobot(robot.id, globalMap.robots.values())
        robot.posHistory.append(robot.position)
        robot.velHistory.append(robot.velocity)
        print robot.id, robot.position, robot.velocity
      else:
        print "robot",robot.id,"is in goal"

    time += dt
    counter += 1

  # Simulation has ended, now prepare data for plotting
  bestTime = float('Inf')
  bestRobot = -1

  for robot in globalMap.robots.values():
    x, y, vx, vy, vmag, vang = [], [], [], [], [], []

    for pos in robot.posHistory:
      x.append(pos[0])
      y.append(pos[1])
    for vel in robot.velHistory:
      vx.append(vel[0])
      vy.append(vel[1])
      vmag.append(np.sqrt(vel[0]**2+vel[1]**2))
      vang.append(np.arctan2(vel[1],vel[0])*180./np.pi)

    robot.histx = x
    robot.histy = y
    robot.histvx = vx
    robot.histvy = vy
    robot.histvmag = vmag
    robot.histvang = vang
    robot.timefinish = random.random()

    if robot.endTime - robot.startTime < bestTime:
      bestRobot = robot.id
      bestTime = robot.endTime - robot.startTime

   
  endTimeProgram = timepackage.time()
  print "Serial Time",endTimeProgram-startTimeProgram
  print "Best robot on process " + str(rank) + ": " + str(bestRobot) + ", Time: " + str(bestTime)


'''
  # Start plotting
  plt.figure()

  for robot in globalMap.robots:
    if robot.id == bestRobot:
      plt.plot(robot.histx,robot.histy,'g-',linewidth=4,alpha=.9)
    plt.plot(robot.histx,robot.histy,'b-',linewidth=2,alpha=1./maxNumRobots*4)
  
  plt.plot(globalMap.robots[bestRobot].histx,globalMap.robots[bestRobot].histy,'g-',linewidth=4,alpha=.9)
  ax1 = plt.gca()
  plt.axis('scaled')
  ax1.set_xlim([0,mapDim[0]])
  ax1.set_ylim([0,mapDim[1]])
  
  for robot in globalMap.robots:
    x = robot.histx
    y = robot.histy
    dotGap = int(np.round(np.float(len(x))/100.))
    xdots = x[0::dotGap]
    ydots = y[0::dotGap]
    if robot.id == bestRobot:
      plt.plot(xdots,ydots,'k.',markersize=3)

  # Add plots of start, goal, and obstacles
  goalCircle = plt.Circle(goalPosition,goalTolerance,color='g')
  ax1.add_artist(goalCircle)
  startCircle = plt.Circle(startPos,goalTolerance,color='b')
  ax1.add_artist(startCircle)
  obstacleCircle1 = plt.Circle(circlePosition,circleRadius,color='r')
  ax1.add_artist(obstacleCircle1)
  rigidCircle = plt.Circle(rigidPos,rigidRad,color='y')
  ax1.add_artist(rigidCircle)
  ax1.set_xlabel('x')
  ax1.set_ylabel('y')


  # In a new plot, graph velocity magnitude and angle
  plt.figure()
  plt.subplot(211)  
  for robot in globalMap.robots:
    plt.plot(robot.histvmag)

  plt.gca().set_ylabel('Speed')
  plt.subplot(212)

  for robot in globalMap.robots:
    plt.plot(robot.histvang)
  plt.gca().set_ylim([-180,180])
  plt.gca().set_xlabel('Time step')
  plt.gca().set_ylabel('Angle from (1,0) (Degrees)')
  plt.show()
'''
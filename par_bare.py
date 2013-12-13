import numpy as np
import matplotlib.pyplot as plt
import time as timepackage
import math
import random
from mpi4py import MPI

# Global variables, can be used by any process
dt = 0.1
time = 0
goalTolerance=6.0

# Map defines the physical space, including dimensions, all robots, obstacles, goal position, etc.
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
      self.robots[robotId].endTime = time


class Robot:
  def __init__(self, position, velocity, velocityMagAng, robotId, start, slowFactor=0.05):
    self.position = position
    self.velocity = velocity
    self.finished = False
    self.id = robotId
    self.V0 = math.sqrt(velocity[0]**2 + velocity[1]**2)
    self.slowFactor = slowFactor
    self.startTime = start
    self.endTime = float('Inf')
    self.startAngle = velocityMagAng[1]
    self.startMag = velocityMagAng[0]

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


  # Initialize variables which define the map
  if rank==0:
    startTimeProgram = timepackage.time()
  startPos = (25,40)
  vMagnitude = 3

  goalPosition = (450,100)
  mapDim = (500,200)
  endTime = 400
  totalRobots = 512
  maxNumRobots = np.ceil(float(totalRobots)/float(comm.Get_size()))
  #maxNumRobots = 16
  
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
  if rank==0:
    startTimeComputation = timepackage.time()
  # Run simulation while until time has expired or all robots are in the goal
  for i in xrange(int(maxNumRobots)):
    #if counter % 5 is 0 and len(globalMap.robots) < maxNumRobots:
    angle = random.random() * 180
    Vx = math.sin(math.radians(angle)) * vMagnitude
    Vy = math.cos(math.radians(angle)) * vMagnitude
    
    rid = len(globalMap.robots)  
    globalMap.addRobot(Robot(startPos, (Vx, Vy),(vMagnitude,angle), rid, time))
  if rank==0:
    startTimeComputation2 = timepackage.time()
  while time < globalMap.endTime and not allRobotsInGoal(globalMap):
    # Release a new robot every 5 iterations
    # if counter % 5 is 0 and len(globalMap.robots) < maxNumRobots:
    #   angle = random.random() * 180
    #   Vx = math.sin(math.radians(angle)) * vMagnitude
    #   Vy = math.cos(math.radians(angle)) * vMagnitude

    #   rid = len(globalMap.robots)  
    #   globalMap.addRobot(Robot(startPos, (Vx, Vy),(vMagnitude,angle), rid, time))

    # Update position and velocity of the robot taking into account all obstacles and other robots
    for robot in globalMap.robots:

      if robot.finished == False:
        globalMap.updatePosition(robot.id)
        globalMap.updateVelocity(robot.id)
        #robot.posHistory.append(robot.position)
        #robot.velHistory.append(robot.velocity)
        #print robot.position, robot.velocity
      #else:
      #  print "robot",robot.id,"is in goal"

    time += dt
    counter += 1

  # Simulation has ended, now prepare data for plotting
  bestTime = float('Inf')

  for robot in globalMap.robots:
    if robot.endTime - robot.startTime < bestTime:
      bestRobot = robot.id
      bestTime = robot.endTime - robot.startTime
  print "bestRobot",bestRobot,"bestTime",bestTime
  bestAngle = globalMap.robots[bestRobot].startAngle
  print "bestAngle",bestAngle
  #collectionBuffer= np.zeros(comm.Get_size())
  #bestAngleArray=np.array([bestAngle],dtype=np.float64)
  #comm.Gather(bestAngleArray,collectionBuffer,root=0)
  bestRobotArray = np.array([bestTime,bestAngle])
  collectionBuffer = np.zeros((comm.Get_size(),2),np.float64)
  comm.Gather(bestRobotArray,collectionBuffer,root=0)
  comm.Barrier()
  if rank==0:
    collectionBuffer = collectionBuffer[collectionBuffer[:,0].argsort()]
    #print "test",collectionBuffer
    print "total number of robots",totalRobots
    print "number of processors",comm.Get_size()
    #print "best robot initial angle",np.min(bestAngleArray)
    print "best robot time",collectionBuffer[0,0],"initial angle",collectionBuffer[0,1],"degrees"
    endTimeProgram = timepackage.time()
    print "Parallel Time with setup",endTimeProgram-startTimeProgram
    print "Parallel Time without setup",endTimeProgram-startTimeComputation2


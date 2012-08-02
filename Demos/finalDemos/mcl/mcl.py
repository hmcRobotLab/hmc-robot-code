import roslib; roslib.load_manifest("irobot_mudd")
import rospy
import cv2
from random import random
from math import pi,cos,sin
import itertools

testMap = "testmap.bmp"

SCANFOV = 57.5
NUMINSCAN = 360
MAXLASER = 11.0
TRACESTEP = 1
HIT_THRES = 210

class Particle:
  def __init__(self,x,y,theta,score = 0):
    self.x = x
    self.y = y
    self.theta = theta
    self.score = score

  def move(self,dx,dy,dth):
    self.x += dx
    self.y += dy
    self.theta += dth
    self.theta %= 2*pi

  
class Mcl:
  def __init__(self,mapName,mapRes,initPose = [0,0,0],numParticles = 1000,mapBounds=[25,25],numRays = 10,fov=pi):
    self.lastRPose = initPose
    self.numParticles = numParticles
    self.particles = []
    self.maxWeight = 0
    self.boundX = mapBounds[0]
    self.boundY = mapBounds[1]
    self.gMap = cv2.imread(mapName)
    self.mapRes = mapRes
    self.numRays = numRays
    self.fov = fov
    # start with random particles
    for x in range( self.numParticles ):
      self.particles.append(Particle(random()*self.boundX,random()*self.boundY,random()*2*pi))

  def rayTrace(self,x,y,theta):
    rays = []
    angStep = self.fov / self.numRays
    for x in range( self.numRays) :
      rays.append(self.getDist(x,y,theta-self.fov/2+angStep * x))
    return rays

  def getDist(self,x,y,theta):
    # returns distance measured starting from global x y heading
    r = 0
    maxPixels = MAXLASER / self.mapRes
    while r < maxPixels:
      mapX = int(r * cos(theta) + x)
      mapY = int(r * sin(theta) + y)
      print mapX
      print mapY
      if self.gMap[mapX,mapY].sum() > HIT_THRES:
        return self.mapRes
      self.gMap[mapX,mapY] = (0,255,0)
      r += TRACESTEP

    return r * self.mapRes

  def score(self,myScan,gScan):
    # score my scan with the given scan
    score = 0
    for trace,real in itertools.izip(myScan,gScan):
      score += abs( real - trace )

    return score

  def moveParticles(self,currX,currY,currTh):
    dx = currX - self.lastRPose[0]
    dy = currY - self.lastRPose[1]
    dTh = currTh - self.lastRPose[2]
    self.lastRPose = [currX,currY,currTh]
    for particle in self.particles:
      particle.move(dx,dy,dth)

  def scoreParticles(self,currScan):
    self.maxWeight = 0
    for particle in self.particles:
      particle.score = self.score(self.rayTrace(particle.x,particle.y,particle.theta,),currScan)
      if particle.score > self.maxWeight:
        self.maxWeight = particle.score

  def cullParticles(self):
    # from Sebastian Thrun's CS373 on udacity.com
    newParticles = []
    index = int(random() * self.numParticles)
    beta = 0.0
    for i in range(self.numParticles):
      beta += random() * 2.0 * self.maxWeight
      while beta > self.particles[i].score:
        beta -= self.particles[i].score
        index = (index + 1) % self.numParticles
      newParticles.append(self.particles[index])
    self.particles = newParticles




def test():
  #map = [[255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255], \
  #       [225,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,225,225,225,225,225,225], \ 
  #       [225,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,225,225,225,225,225,225], \ 
  #       [225,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,225,225,225,225,225,225], \ 
  #       [225,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,225,225,225,225,225,225], \ 
  #       [225,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,225,225,225,225,225,225], \ 
  #       [225,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,225,225,225,225,225,225], \ 
  #       [225,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,225,225,225,225,225,225], \ 
  #       [225,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,225,225,225,225,225,225], \ 
  #       [225,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,225,225,225,225,225,225], \ 
  #       [225,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,225], \ 
  #       [225,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,225], \ 
  #       [225,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,225], \ 
  #       [225,255,255,255,255,255,255,255,225,225,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,225], \ 
  #       [225,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,225], \ 
  #       [225,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,225], \ 
  #       [225,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,225], \ 
  #       [225,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,225], \ 
  #       [225,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,225,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,225], \ 
  #       [225,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,225,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,225], \ 
  #       [225,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,225,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,225], \ 
  #       [225,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,225,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,225], \ 
  #       [225,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,225,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,225], \ 
  #       [225,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,225,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,225], \ 
  #       [255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255]]
  fakeRobot = Particle(75.0,75.0,pi)
  mcl =  Mcl(testMap,0.05) # 0.05 meters per pixel
  cv2.namedWindow("map")
  cv2.waitKey()
  mcl.gMap[150][150] = (255,0,0)
  mcl.gMap[150][151] = (255,0,0)
  mcl.gMap[151][150] = (255,0,0)
  mcl.gMap[151][151] = (255,0,0)
  cv2.imshow("map",mcl.gMap)
  cv2.waitKey()

  scan = mcl.rayTrace(fakeRobot.x,fakeRobot.y,fakeRobot.theta)
  print scan
  cv2.waitKey()
  print "done"
  cv2.waitKey()
  print "done"
  while True:
    pass

  #print "Scoring particles"
  #mcl.scoreParticles(scan)
  #print "Culling particles"
  #mcl.cullParticles()

  #fakeRobot.move(.123,.456,.1)
  #print "Moving particles"
  #mcl.moveParticles(.123,.456,.1)
  #scan = mcl.rayTrace(fakeRobot.x,fakeRobot.y,fakeRobot.theta)
  #print "Scoring particles"
  #mcl.scoreParticles(scan)


if __name__ == "__main__":
  test()

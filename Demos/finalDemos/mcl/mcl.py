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
  def __init__():
    self.x = 0
    self.y = 0
    self.theta = 0
    self.score = 0

  def __init__(x,y,theta,score = 0):
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
  def __init__(mapName,mapRes,initPose = [0,0,0],numParticles = 1000,mapBounds=[25,25],numRays = 10,fov=pi):
    self.lastRPose = initPose
    self.numPartices = numPartices
    self.particles = []
    self.maxWeight = 0
    self.boundX = mapBounds[0]
    self.boundY = mapBounds[1]
    self.gMap = cv.LoadImageM(mapName)
    self.mapRes = mapRes
    self.numRays = numRays
    self.fov = fov
    # start with random particles
    for x in range self.numParticles:
      self.partices.append(Particle(random.random()*MAPBOUNDX,random.random()*MAPBOUNDY,random.random()*2*pi))

  def rayTrace(self,x,y,theta):
    rays = []
    angStep = self.fov / self.numRays
    for x in range self.numRays:
      rays.append(self.getDist(x,y,theta-self.fov/2+angStep * x))
    return rays



  def getDist(self,x,y,theta):
    # returns distance measured starting from global x y heading
    r = 0
    maxPixels = MAXLASER / self.mapRes
    while r < maxPixels:
      mapX = float(r * cos(theta) + x)
      mapY = float(r * sin(theta) + y)
      if self.gMap[mapX][mapY] > HIT_THRES:
        return self.mapRes
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
      particle.score = self.score(self.rayTrace(particle.x,particle,y,particle.theta,),currScan)
      if particle.score > self.maxWeight:
        self.maxWeight = particle.score

  def cullParticles(self):
    # from Sebastian Thrun's CS373 on udacity.com
    newParticles = []
    index = int(random.random() * numParticles)
    beta = 0.0
    for i in range(numParticles):
      beta += random.random() * 2.0 * self.maxWeight
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
  fakeRobot = Particle(5.0,5.0,pi)
  mcl =  Mcl(testmap,0.05) # 0.05 meters per pixel

  scan = mcl.rayTrace(fakeRobot.x,fakeRobot.y,fakeRobot.theta)
  print "Scoring particles"
  mcl.scoreParticles(scan)
  print "Culling particles"
  mcl.cullParticles()

  fakeRobot.move(.123,.456,.1)
  print "Moving particles"
  mcl.moveParticles(.123,.456,.1)
  scan = mcl.rayTrace(fakeRobot.x,fakeRobot.y,fakeRobot.theta)
  print "Scoring particles"
  mcl.scoreParticles(scan)


if __name__ == "__main__":
  test()

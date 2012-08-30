
import cv2
from random import random,gauss
from math import pi,cos,sin,exp,sqrt
import itertools
import copy

testMap = "testmap.bmp"
HIT_THRES = 1
TRACESTEP = 1


class Particle:
  def __init__(self,x,y,theta,score = 0,transNoise = 0.1,rotNoise = 0.04,laserNoise = 0.5):
    self.x = x
    self.y = y
    self.theta = theta
    self.score = score
    self.transNoise = transNoise
    self.rotNoise = rotNoise
    self.laserNoise = laserNoise

  def move(self,dist,ang):
    self.theta += gauss(float(ang), self.rotNoise)
    self.theta %= 2*pi
      
    dist = gauss(float(dist), self.transNoise)
    self.x += (cos(self.theta) * dist)
    self.y += (sin(self.theta) * dist)

  def __repr__(self):
    return str([self.x,self.y,self.theta])


class Mcl:
  def __init__(self,mapName,mapRes,numParticles = 350,numRays = 5,fov=pi/2,maxLaser = 23.0):
    self.numParticles = numParticles
    self.particles = []
    self.gMap = cv2.cv.LoadImageM(mapName,0) # seems to be faster for scoring then imread..
    self.drawMap = cv2.imread(mapName)
    self.drawMapStatic = cv2.imread(mapName)
    self.mapRes = mapRes
    self.numRays = numRays
    self.fov = fov
    self.maxLaser = maxLaser
    self.mem = {}

  def addParticles(self,pose=None):
    # add particles randomly to the map
    # this could be improved by doing some 'uniform sampling'
    # as described in prob. robots
    for x in range( self.numParticles ):
      if pose:
        self.particles.append(Particle(pose[0],pose[1],pose[2],transNoise = 0.1, rotNoise = 0.04, laserNoise = self.mapRes * 2))
      else:
        self.particles.append( self.randParticle())

  def randParticle(self):
    return Particle(random()*self.gMap.width * self.mapRes ,random()*self.gMap.height * self.mapRes,\
        random()*2*pi,transNoise=0.1,rotNoise=0.1,laserNoise = self.mapRes * 2)

  def getDist(self,x,y,theta,draw=False):
    r = 0
    maxPixels = self.maxLaser / self.mapRes
    mapX=0
    mapY=0
    while r < maxPixels:
      mapX = int(r * cos(theta) + x)
      mapY = int(r * sin(theta) + y)
      if mapX > self.gMap.width - 1 or mapX < 0:
        return r * self.mapRes
      elif mapY > self.gMap.height - 1 or mapY < 0:
        return r * self.mapRes
      elif self.gMap[mapY,mapX] < HIT_THRES:
        return r * self.mapRes 
      if draw:
        self.drawMap[mapY,mapX] = (0,255,0)
      r += TRACESTEP

    return r * self.mapRes

  def rayTrace(self,particle,draw=False):
    # ray trace until a wall is hit.  This is slow but its how ros-amcl
    # does it.  It could be improved by doing a 'bresenham' line and could
    # be sped up by using an octree structure to skip over large blank areas
    # x,y in pixels

    # this should help a bit especially after resampling..
    key = (int(particle.x/self.mapRes),int(particle.y/self.mapRes),round(particle.theta,1))
    if self.mem.has_key(key):
      return self.mem[key]
    
    x = particle.x / self.mapRes
    y = particle.y / self.mapRes
    scan = []
    angStep = self.fov / (self.numRays-1)
    for ang in range( self.numRays) :
      scan.append(self.getDist(x,y,particle.theta-self.fov/2+angStep * ang,draw))

    # add some noise to the scan
    for x in range(len(scan)):
      if scan[x] < self.maxLaser:
        scan[x] = gauss(scan[x],particle.laserNoise)
    self.mem[key] = scan
    return scan

  def moveParticles(self,dist,th):
    for particle in self.particles:
      particle.move(dist,th)

  def scoreParticle(self,particle,currScan):
    if particle.x/self.mapRes > self.gMap.width - 1 or particle.x < 0:
      particle.score = 0
    elif particle.y/self.mapRes > self.gMap.height - 1 or particle.y < 0:
      particle.score = 0
    elif self.gMap[particle.y/self.mapRes,particle.x/self.mapRes] < HIT_THRES:
      particle.score = 0
    else:
      myScan = self.rayTrace(particle)
      weight = 1.0
      for trace,real in itertools.izip(myScan,currScan):
        # probabilty of trace being the same as the real reading
        weight *= self.Gaussian(trace,particle.laserNoise,real)
      particle.score = weight

  def scoreParticles(self,currScan):
    for particle in self.particles:
      self.scoreParticle(particle,currScan)

  def Gaussian(self, mu, sigma, x):
    # from Sebastian Thrun's CS373 on udacity.com  
    # calculates the probability of x for 1-dim Gaussian with mean mu and var. sigma
    return exp(- ((mu - x) ** 2) / (sigma ** 2) / 2.0) / sqrt(2.0 * pi * (sigma ** 2))

  def resample(self):
    # Resample with replacement according the weights of the paricles
    weightSum = 0.0
    maxWeight = 0.0

    for particle in self.particles:
      if particle.score > maxWeight:
        maxWeight = particle.score
      weightSum += particle.score 

    if weightSum == 0:
      self.particles = []
      self.addParticles()
    else:
      for particle in self.particles:
        particle.score = particle.score/weightSum

      avg = weightSum / len(self.particles)
      print "Average: " + str(avg)

      # TODO: Make it adaptive as described in prob robots
      # numRand = int( (1 - avg - maxWeight) * self.numParticles)/16
      # numRand = 0
      # for i in range(numRand):
      #   newParticles.append( self.randParticle() )

      # from Sebastian Thrun's CS373 on udacity.com
      newParticles = []
      maxWeight = maxWeight / weightSum
      index = int(random() * len(self.particles))
      beta = 0.0

      for i in range(self.numParticles):
        beta += random() * 2.0 * maxWeight
        while beta > self.particles[index].score:
          beta -= self.particles[index].score
          index = (index + 1) % len(self.particles)
        newParticles.append(copy.deepcopy(self.particles[index]))
      self.particles = newParticles

  def drawParticle(self,particle,color = (0,0,255)):
    mapX = int(particle.x / self.mapRes)
    mapY = int(particle.y / self.mapRes)
    r=5
    rx = mapX + int(r * cos(particle.theta))
    ry = mapY + int(r * sin(particle.theta) )
    cv2.circle(self.drawMap,(mapX,mapY),3,color)
    cv2.line(self.drawMap,(mapX,mapY),(rx,ry),color)

  def resetDraw(self):
    self.drawMap = copy.deepcopy(self.drawMapStatic)

  def drawAll(self):
    for p in self.particles:
      self.drawParticle(p)


def testGaussMove():
  cv2.namedWindow("map")
  mcl =  Mcl(testMap,0.20) # 0.20 meters per pixel
  mcl.addParticles([35.0,50.0,0])
  while True:
    mcl.resetDraw()
    mcl.moveParticles(.5,0)
    mcl.drawAll()
    cv2.imshow("map",mcl.drawMap)
    cv2.waitKey(23)

def main():
  cv2.namedWindow("map")
  fakeRobot = Particle(35.0,50.0,0)
  odomOnly = Particle(35.0,50.0,0,transNoise = 0,rotNoise=0)
  mcl =  Mcl(testMap,0.20) # 0.20 meters per pixel
  #mcl.numParticles = 5000
  mcl.addParticles([35.0,50.0,0])
  #mcl.addParticles()
  mcl.drawParticle(fakeRobot,(0,255,0))
  mcl.drawAll()
  cv2.imshow("map",mcl.drawMap)
  cv2.waitKey(5)
  while True:
    mcl.resetDraw()

    # some random movement
    possX = 0
    possY = 0
    rDist = 0
    rTh = 0
    goodP = False
    tried = 0
    backup = 0.0
    while not goodP: 
      rDist = (random() * .8) - backup
      rTh = (random() * .6) - .3
      rTh %= 2*pi 
      possTh = fakeRobot.theta + rTh
      possX = fakeRobot.x + (cos(possTh) * rDist)
      possY = fakeRobot.y + (sin(possTh) * rDist)
      if mcl.gMap[int(possY/mcl.mapRes),int(possX/mcl.mapRes)] > HIT_THRES:
        scan = mcl.rayTrace(Particle(possX,possY,possTh),True)
        if min(scan) > 3.0:
          goodP = True

      tried +=1 
      if tried > 40:
        tried = 0
        backup += .2
    ####

    fakeRobot.move(rDist,rTh)
    odomOnly.move(rDist,rTh)
    mcl.moveParticles(rDist,rTh)
    scan = mcl.rayTrace(fakeRobot)
    mcl.scoreParticles(scan)
    #mcl.numParticles = 400
    mcl.resample()
    mcl.drawParticle(fakeRobot,(0,255,0))
    mcl.drawParticle(odomOnly,(255,105,105))
    mcl.drawAll()
    cv2.imshow("map",mcl.drawMap)
    cv2.waitKey(1)



if __name__ == "__main__":
  #testGaussMove()
  main()

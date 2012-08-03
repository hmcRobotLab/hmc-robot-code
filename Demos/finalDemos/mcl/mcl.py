import roslib; roslib.load_manifest("irobot_mudd")
import rospy
import cv2
from random import random,gauss
from math import pi,cos,sin,exp,sqrt
import itertools
import copy

testMap = "testmap.bmp"
HIT_THRES = 1
TRACESTEP = 1
class Particle:
  def __init__(self,x,y,theta,score = 0,transNoise = 0.1,rotNoise = 0.1,laserNoise = 0.5):
    self.x = x
    self.y = y
    self.theta = theta
    self.score = score
    self.transNoise = transNoise
    self.rotNoise = rotNoise
    self.laserNoise = laserNoise

  #def move(self,dx,dy,dth):
  #  self.x += dx
  #  self.y += dy
  #  self.theta += dth
  #  self.theta %= 2*pi

  #def moveGauss(self,dx,dy,dth):
  #  self.x += dx + gauss(0.0,TRANS_NOISE)
  #  self.y += dy + gauss(0.0,TRANS_NOISE)

  #  self.theta += dth + gauss(0.0,ROT_NOISE)
  #  self.theta %= 2*pi

  def move(self,dist,ang):
    self.theta += gauss(float(ang), self.rotNoise)
    self.theta %= 2*pi
      
    dist = gauss(float(dist), self.transNoise)
    self.x += (cos(self.theta) * dist)
    self.y += (sin(self.theta) * dist)

  def __repr__(self):
    return str([self.x,self.y,self.theta])

  
class Mcl:
  def __init__(self,mapName,mapRes,initPose = [0,0,0],numParticles = 150,numRays = 5,fov=pi/2,maxLaser = 23.0):
    self.lastRPose = initPose
    self.numParticles = numParticles
    self.particles = []
    self.gMap = cv2.cv.LoadImageM(mapName,0) # seems to be faster for scoring then imread..
    self.drawMap = cv2.imread(mapName)
    self.drawMapStatic = cv2.imread(mapName)
    self.mapRes = mapRes
    self.numRays = numRays
    self.fov = fov
    self.maxLaser = maxLaser
    # start with random particles
    #self.maxXP = self.gMap.shape[0] 
    #self.maxYP = self.gMap.shape[1] 
    #self.maxX = self.gMap.shape[0] * self.mapRes
    #self.maxY = self.gMap.shape[1] * self.mapRes
    self.maxXP = self.gMap.width
    self.maxYP = self.gMap.height 
    self.maxX = self.gMap.width * self.mapRes
    self.maxY = self.gMap.height * self.mapRes
    self.hitPixels = []

    #self.preCalcHits()
    for x in range( self.numParticles ):
      #self.particles.append( self.randParticle())
      self.particles.append( Particle(45.0,100.0,0) )

  def randParticle(self):
    return Particle(random()*self.maxX,random()*self.maxY,random()*2*pi,transNoise=0.2,rotNoise=0.2,laserNoise = self.mapRes * 2)

  def preCalcHits(self):
    ix,iy = self.gMap.width,self.gMap.height
    for y in range(iy):
      for x in range(ix):
        if self.gMap[x,y] < HIT_THRES:
          self.hitPixels.append((x,y))
    print self.hitPixels

  def getScan(self,particle,draw=False):
    scan = self.rayTrace(particle.x/self.mapRes,particle.y/self.mapRes,particle.theta,draw)
    for x in range(len(scan)):
      if scan[x] < self.maxLaser:
        scan[x] = gauss(scan[x],particle.laserNoise)
    return scan

  def rayTrace(self,x,y,theta,draw=False):
    # x,y in pixels
    rays = []
    angStep = self.fov / (self.numRays-1)
    #print "x: " + str(x) + " y: " + str(y) + " th: " + str(theta)
    for ang in range( self.numRays) :
      #print str(x) + " ",
      rays.append(self.getDist(x,y,theta-self.fov/2+angStep * ang,draw))
    return rays

  def getDist(self,x,y,theta,draw=False):
    # returns distance measured starting from global x y heading in pixels
    r = 0
    maxPixels = self.maxLaser / self.mapRes
    mapX=0
    mapY=0
    while r < maxPixels:
      mapX = int(r * cos(theta) + x)
      mapY = int(r * sin(theta) + y)
      if mapX > self.maxXP -1 or mapX < 0:
        return r * self.mapRes
      elif mapY > self.maxYP -1 or mapY < 0:
        return r * self.mapRes
      elif self.gMap[mapY,mapX] < HIT_THRES:
        return r * self.mapRes 
      #elif (mapX,mapY) in self.hitPixels:
      #  return r * self.mapRes 
      if draw:
        self.drawMap[mapY,mapX] = (0,255,0)
      r += TRACESTEP

    #cv2.line(self.gMap,(int(x),int(y)),(mapX,mapY),(255,0,0))
    return r * self.mapRes

  #def score(self,myScan,gScan):
  #  # score my scan with the given scan
  #  weight = 1.0
  #  #for trace,real in itertools.izip(myScan,gScan):
  #  #  if trace > self.maxLaser and real > self.maxLaser:
  #  #    score += .5
  #  #  elif trace > self.maxLaser:
  #  #    score += 0
  #  #  elif real > self.maxLaser:
  #  #    score += 0
  #  #  else:
  #  #    score += abs(real/trace)

  #
  #  #  #score -= abs( real - trace )

  #  #for reading in myScan:
  #  #  if reading == self.maxLaser:
  #  #    score -= self.maxLaser/2

  #  for trace,real in itertools.izip(myScan,gScan):
  #    weight *= self.Gaussian(abs(real-trace),
  #  return weight

  def moveParticles(self,dist,th):
    for particle in self.particles:
      particle.move(dist,th)

  def vInM(self,x,y):
    return self.gMap[x/self.mapRes,y/self.mapRes]

  def scoreParticle(self,particle,currScan):
    if particle.x > self.maxX or particle.x < 0:
      particle.score = 0
    elif particle.y > self.maxY or particle.y < 0:
      particle.score = 0
    elif self.vInM(particle.y,particle.x) < HIT_THRES:
      particle.score = 0
    else:
      myScan = self.rayTrace(particle.x/self.mapRes,particle.y/self.mapRes,particle.theta)
      weight = 1.0
      for trace,real in itertools.izip(myScan,currScan):
        weight *= self.Gaussian(trace,particle.laserNoise,real)
      particle.score = weight

  def scoreParticles(self,currScan):
    for particle in self.particles:
      self.scoreParticle(particle,currScan)
        

  def Gaussian(self, mu, sigma, x):
    # from Sebastian Thrun's CS373 on udacity.com  
    # calculates the probability of x for 1-dim Gaussian with mean mu and var. sigma
    return exp(- ((mu - x) ** 2) / (sigma ** 2) / 2.0) / sqrt(2.0 * pi * (sigma ** 2))

  def cullParticles(self):
    # from Sebastian Thrun's CS373 on udacity.com
    weightSum = 0.0
    maxWeight = 0.0

    for particle in self.particles:
      if particle.score > maxWeight:
        maxWeight = particle.score
      weightSum += particle.score 

    for particle in self.particles:
      particle.score = particle.score/weightSum
    print "maxWeight " + str(maxWeight)

    newParticles = []
    index = int(random() * self.numParticles)
    beta = 0.0
 
    avg = weightSum / self.numParticles
    

    numRand = int( (1 - avg - maxWeight) * self.numParticles)/8
    maxWeight = maxWeight / weightSum
    print "avg " + str(avg)
    print "numrand " + str(numRand)
    for i in range(numRand):
      newParticles.append( self.randParticle() )

    for i in range(self.numParticles-numRand):
      beta += random() * 2.0 * maxWeight
      while beta > self.particles[index].score:
        #print (i,index,beta,self.particles[index].score)
        beta -= self.particles[index].score
        index = (index + 1) % self.numParticles
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

def testCoorelating():
  mcl =  Mcl(testMap,0.1) # 0.05 meters per pixel
  fakeRobot = Particle(25.0,25.0,pi)
  fakeRobot1 = Particle(16.2,25.2,pi)
  scan = mcl.getScan(fakeRobot)
  mcl.scoreParticle(fakeRobot,scan)
  mcl.scoreParticle(fakeRobot1,scan)
  print fakeRobot1.score
  

def test():
  #mcl.drawAll()
  #cv2.namedWindow("map")
  #cv2.waitKey()
  #cv2.imshow("map",mcl.drawMap)
  #cv2.waitKey()
  #mcl.drawParticle(fakeRobot)
  #cv2.imshow("map",mcl.drawMap)
  #cv2.waitKey()

  #scan = mcl.rayTrace(fakeRobot.x,fakeRobot.y,fakeRobot.theta)
  #cv2.imshow("map",mcl.drawMap)
  #cv2.waitKey()
  #print "scoring"
  #mcl.scoreParticles(scan)
  #cv2.imshow("map",mcl.drawMap)
  #cv2.waitKey()
  #mcl.resetDraw()
  #print "culling"
  #mcl.cullParticles()
  #print "done"
  #mcl.drawAll()
  #cv2.imshow("map",mcl.drawMap)
  #cv2.waitKey()
  #print "done"
  mcl =  Mcl(testMap,0.25) # 0.05 meters per pixel
  fakeRobot = Particle(45.0,100.0,0)
  #while True:
  #  mcl.resetDraw()
  #  fakeRobot.move(.1,0)
  #  print fakeRobot
  #  mcl.drawParticle(fakeRobot,(100,255,100))

  #  cv2.imshow("map",mcl.drawMap)
  #  cv2.waitKey()

  #while True:
  #  mcl.resetDraw()
  #  mcl.moveParticles(0,.1)
  #  mcl.drawAll()
  #  cv2.imshow("map",mcl.drawMap)
  #  print "done"
  #  cv2.waitKey(50)

  while True:
    mcl.resetDraw()
    possX = 0
    possY = 0
    rDist = 0
    rTh = 0
    goodP = False
    while not goodP: 
      rDist = (random() * .8) -.1
      rTh = (random() * .4) - .2
      rTh %= 2*pi 
      possTh = fakeRobot.theta + rTh
      possX = fakeRobot.x + (cos(possTh) * rDist)
      possY = fakeRobot.y + (sin(possTh) * rDist)
      testParticle = Particle(possX,possY,possTh)
      if sum(mcl.getScan(testParticle)) > 45:
        goodP = True
    fakeRobot.move(rDist,rTh)
    print "moving"
    mcl.moveParticles(rDist,rTh)
    scan = mcl.getScan(fakeRobot,True)
    print "scoring"
    mcl.scoreParticles(scan)
    print "culling"
    mcl.cullParticles()
    print "drawing"
    mcl.drawParticle(fakeRobot,(100,255,100))
    mcl.drawAll()
    cv2.imshow("map",mcl.drawMap)
    print "done"
    cv2.waitKey(5)

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

USE_DRONE = True

if(USE_DRONE):
  import roslib; roslib.load_manifest('ardrone_mudd')
  from ardrone_mudd.srv import *
  from ardrone_mudd.msg import *
  from sensor_msgs.msg import *
  import rospy
import time,sys,random,cv,cv_bridge,math

class DroneData:
  altitude = 542
  vx = 3
  vy = 4
  vz = 0
  phi = 100
  theta = 4242
  psi = 3064
  state = 789

def updateKey():
  global lastsent
  maxpower = .15
  
  # wait for keypress  
  char = cv.WaitKey()

  sflag = 0
  sphi = 0
  stheta = 0
  sgaz = 0
  syaw = 0
  sgaz = 0 

  if char == ord(' '):
      helistr = "land";
  elif char == ord('r'):
      helistr = "reset"
  elif char == ord('t'):
      helistr = "takeoff"
  else:
      if char == ord('w'):
          sflag = 1
          stheta = -maxpower
      elif char == ord('x'):
          sflag = 1
          stheta = maxpower
      elif char == ord('a'):
          sflag = 1
          sphi = -maxpower  
      elif char == ord('d'):
          sflag = 1
          sphi = maxpower  
      elif char == ord('e'):
          sflag = 1
          stheta = -math.sqrt(maxpower/2)
          sphi = math.sqrt(maxpower/2)  
      elif char == ord('z'):
          sflag = 1
          stheta = math.sqrt(maxpower/2)
          sphi = -math.sqrt(maxpower/2)  
      elif char == ord('q'):
          sflag = 1
          stheta = -math.sqrt(maxpower/2)
          sphi = -math.sqrt(maxpower/2)  
      elif char == ord('c'):
          sflag = 1
          stheta = math.sqrt(maxpower/2)
          sphi = math.sqrt(maxpower/2)  
      elif char == ord('s'):
          send("heli 1 0 0 0 0")
          rospy.sleep(.05)
          sflag = 0
      else:
          send(lastsent)
          return;

      helistr = "heli %i %.3f %.3f %.3f %.3f" \
        % (sflag,sphi,stheta,sgaz,syaw)
  send(helistr)

def updateGui():
    x = 0

def send(command):
  global lastsent
  lastsent = command
  heli(command)

def navDataUpdate(data):
  global commands
  global lastsent
  global helistr
  global droneData

  droneData = data
  sys.stdout.write("\r\t [alt: %i] \t [phi: %i psi: %i theta: %i] \t [vx: %i vy: %i vz: %i] \t [bat: %i  state: %i]              \n" \
      %
      (data.altitude,data.phi,data.psi,data.theta,data.vx,data.vy,data.vz,data.batLevel,data.ctrlState))
  sys.stdout.write("\rLast sent: %s                  \n" % (lastsent))
  sys.stdout.write("\033[A")
  sys.stdout.write("\033[A")
  sys.stdout.flush()

def main():
  global heli
  global lastsent
  lastsent = "none"
  if(USE_DRONE):
    cv.NamedWindow('control')
    cv.MoveWindow('control', 200, 500)
    cv.StartWindowThread()
    rospy.init_node("droneController")
    print "Connecting to droneControl service"
    rospy.wait_for_service("droneControl")
    heli = rospy.ServiceProxy("droneControl", Control) #add persistent
    print "\r Connecting to navData service"
    rospy.Subscriber("navData",navData, navDataUpdate, queue_size=1)
    print "\r Connected to services\n"
    while True:
        updateKey()
        updateGui()
  #else:
  #  fd = FakeData()
  #  def heli(command):
  #    commands = command.split()
  #    fd.altitude += 40 * float(commands[4])
  #  print "USE_DRONE SET TO FALSE"
  #  while True:
  #    navDataUpdate(fd)
  #    if random.randint(0,1):
  #      fd.altitude += random.randint(-10,10)
  #    time.sleep(.05)
    

if __name__ == "__main__":
  main()

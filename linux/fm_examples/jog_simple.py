from fairino import Robot
import time
# A connection is established with the robot controller. A successful connection returns a robot object
robot = Robot.RPC('192.168.58.2')

# Robot single axis point
robot.StartJOG(0,1,0,20.0,20.0,30.0)     # Single joint motion, StartJOG is a non blocking command, and other motion commands (including StartJOG) received during motion will be discarded
time.sleep(1)
#Robot single axis jog deceleration stop
# ret = robot.StopJOG(1)
# print(ret)
#Immediate stop of robot single axis jog
robot.ImmStopJOG()
robot.StartJOG(0,2,1,20.0)
time.sleep(1)
robot.ImmStopJOG()
robot.StartJOG(0,3,1,20.0)
time.sleep(1)
robot.ImmStopJOG()
robot.StartJOG(0,4,1,20.0,vel=40)
time.sleep(1)
robot.ImmStopJOG()
robot.StartJOG(0,5,1,20.0,acc=50)
time.sleep(1)
robot.ImmStopJOG()
robot.StartJOG(0,6,1,20.0,20.0,30.0)
time.sleep(1)
robot.ImmStopJOG()
# Base coordinate
robot.StartJOG(2,1,0,20.0)  #Jogging in the base coordinate system
time.sleep(1)
#Robot single axis jog deceleration stop
# robot.StopJOG(3)
#Immediate stop of robot single axis jog
robot.ImmStopJOG()
robot.StartJOG(2,1,1,20.0)
time.sleep(1)
robot.ImmStopJOG()
robot.StartJOG(2,2,1,20.0)
time.sleep(1)
robot.ImmStopJOG()
robot.StartJOG(2,3,1,20.0)
time.sleep(1)
robot.ImmStopJOG()
robot.StartJOG(2,4,1,20.0)
time.sleep(1)
robot.ImmStopJOG()
robot.StartJOG(2,5,1,20.0)
time.sleep(1)
robot.ImmStopJOG()
robot.StartJOG(2,6,1,20.0)
time.sleep(1)
robot.ImmStopJOG()

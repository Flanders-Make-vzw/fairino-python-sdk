from fairino import Robot
import time

# Connect to robot
robot = Robot.RPC("192.168.58.2")  # Replace with your robot's IP

# Basic robot operations
robot.Mode(0)           # Set to automatic mode
robot.RobotEnable(1)    # Enable robot
robot.ResetAllError()   # Clear any errors

#Gripper move parameters
#index = 2 -> gripperID
#pos = int [0-100] -> 0 = open, 100 = closed
#vel = int [0-100] -> 0 = slow, 100 = fast
#force = int [0-100]
#maxtime = int [0-30000] -> milliseconds
#block = int [0,1]
#type = int[0,1]
#rotNum = int
#rotVel = int [0-100]
#rotTorque = int [0-100]
#return 0 -> 0 is completed

#Open gripper
gripperresult = robot.MoveGripper(index=2, pos=0, vel=50, force=10, maxtime=1000, block=0, type=0, rotNum=0, rotVel=0,
                                  rotTorque=0)
print(gripperresult)
time.sleep(5)
#Close gripper
gripperresult = robot.MoveGripper(index=2, pos=100, vel=50, force=10, maxtime=1000, block=0, type=0, rotNum=0, rotVel=0,
                                  rotTorque=0)
print(gripperresult)
time.sleep(5)

# Close connection
robot.CloseRPC()
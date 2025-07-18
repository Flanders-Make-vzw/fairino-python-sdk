from fairino import Robot
import time
# A connection is established with the robot controller. A successful connection returns a robot object
robot = Robot.RPC('192.168.58.2')
ret = robot.RobotEnable(0)   # Unable the robot
print("Enabled on the robot ", ret)


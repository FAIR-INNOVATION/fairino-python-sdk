from fairino import Robot
import time
# A connection is established with the robot controller. A successful connection returns a robot object
robot = Robot.RPC('192.168.58.2')

print("WaitAI start")
error = robot.WaitAI(id=0,sign=0,value=50,maxtime=5000,opt=2)
print("WaitAI return ",error)

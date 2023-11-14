from fairino import Robot
import time
# A connection is established with the robot controller. A successful connection returns a robot object
robot = Robot.RPC('192.168.58.2')
# P1=[-321.821, 125.694, 282.556, 174.106, -15.599, 152.669]
name = 'tpd2023'
blend = 1
ovl = 100.0
ret = robot.LoadTPD(name)  # Trajectory preloading
print("Trajectory preloading",ret)
ret,P1 = robot.GetTPDStartPose(name)   # ObtainTPD start pose
print ("ObtainTPD start pose ",ret," start pose ",P1)
ret = robot.MoveL(P1,0,0)       # Move to the start pose
print("Move to the start pos",ret)
time.sleep(10)
ret = robot.MoveTPD(name, blend, ovl)  # Trajectory reproduction
print("Trajectory reproduction ",ret)


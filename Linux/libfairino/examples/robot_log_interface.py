from fairino import Robot
import time

# A connection is established with the robot controller. A successful connection returns a robot object
robot = Robot.RPC('192.168.58.2')

robot.LoggerInit()
robot.SetLoggerLevel(4)

i=0
while(i<20):
    ret = robot.GetRobotInstallAngle()
    ret = robot.GetActualJointPosDegree()
    ret = robot.GetActualJointPosRadian()
    ret = robot.GetActualJointSpeedsDegree()
    ret = robot.GetTargetTCPCompositeSpeed()
    ret = robot.GetActualTCPCompositeSpeed()
    time.sleep(0.1)
    i=i+1

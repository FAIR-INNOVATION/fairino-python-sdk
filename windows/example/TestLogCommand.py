from fairino import Robot
import time
# A connection is established with the robot controller. A successful connection returns a robot object
robot = Robot.RPC('192.168.58.2')



def log(self):
    """日志输出"""
    robot.LoggerInit(file_path="D://zUP/fairino.log")
    robot.SetLoggerLevel(lvl=3)
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


log(robot)

from fairino import Robot
import time
# A connection is established with the robot controller. A successful connection returns a robot object
import time
import threading

# 与机器人控制器建立连接，连接成功返回一个机器人对象
robot = Robot.RPC('192.168.58.2')


def WeaveAngle(self):
    startdescPose = [146.273, -208.110, 270.102, 177.523, -3.782, -158.101]
    startjointPos = [98.551, -128.309, 127.341, -87.490, -94.249, -13.208]
    enddescPose = [146.272, -476.204, 270.102, 177.523, -3.781, -158.101]
    endjointPos = [93.931, -89.722, 102.216, -101.300, -94.359, -17.840]
    exaxisPos = [0, 0, 0, 0]
    offdese = [0, 0, 0, 0, 0, 0]
    robot.WeaveSetPara(0, 0, 2.000000, 0, 10.000000, 0.000000, 0.000000, 0, 0, 0, 0, 0, 0, 0)
    robot.MoveL(desc_pos=startdescPose,tool=2,user=0,vel=80)
    robot.WeaveStart(0)
    robot.MoveL(desc_pos=enddescPose,tool=2,user=0,vel=80)
    robot.WeaveEnd(0)

    robot.WeaveSetPara(0, 0, 2.000000, 0, 10.000000, 0.000000, 0.000000, 0, 0, 0, 0, 0, 0, 30)
    robot.MoveL(desc_pos=startdescPose,tool=2,user=0,vel=80)
    robot.WeaveStart(0)
    robot.MoveL(desc_pos=enddescPose,tool=2,user=0,vel=80)
    robot.WeaveEnd(0)
    return 0

WeaveAngle(robot)
from time import sleep

from fairino import Robot
from ctypes import sizeof
# A connection is established with the robot controller. A successful connection returns a robot object
# 与机器人控制器建立连接,连接成功返回一个机器人对象
robot = Robot.RPC('192.168.58.2')

import time

def TestInverseKinExaxis(self):
    desc = [99.957, -0.002, 29.994, -176.569, -6.757, -167.462]
    exaxis = [100.0, 0.0, 0.0, 0.0]
    jointPos = [0.0] * 6
    offsetPos = [0.0] * 6

    rtn,pkg = robot.GetRobotRealTimeState()
    toolnum = pkg.tool
    workPcsNum = pkg.user
    rtn, jointPos = robot.GetInverseKinExaxis(0, desc, exaxis, toolnum, workPcsNum)
    print(
        f"GetInverseKinExaxis joint is {jointPos[0]},{jointPos[1]},{jointPos[2]},{jointPos[3]},{jointPos[4]},{jointPos[5]}")

    robot.ExtAxisMove(exaxis, 100, -1)
    robot.MoveJ(joint_pos=jointPos, desc_pos=desc, tool=toolnum, user=workPcsNum, vel=100.0, acc=100.0, ovl=100.0, exaxis_pos=exaxis, blendT=-1, offset_flag=0, offset_pos=offsetPos)

    robot.CloseRPC()
    return 0


def TestServoCart(self):
    desc_pos_dt = [83.00800, 50.525000, 29.246, 179.629, -7.138, -166.975]
    exaxis = [100.0, 0.0, 0.0, 0.0]
    pos_gain = [0.0] * 6
    mode = 0
    vel = 0.0
    acc = 0.0
    cmdT = 0.001
    filterT = 0.0
    gain = 0.0
    flag = 0
    count = 5000

    robot.SetSpeed(20)

    while count:
        rtn = robot.ServoCart(mode, desc_pos_dt, exaxis, pos_gain, acc, vel, cmdT, filterT, gain)
        print(f"ServoCart rtn is {rtn}")
        count -= 1
        desc_pos_dt[0] += 0.01
        exaxis[0] += 0.01

    robot.CloseRPC()
    return 0

# TestInverseKinExaxis(robot)
TestServoCart(robot)
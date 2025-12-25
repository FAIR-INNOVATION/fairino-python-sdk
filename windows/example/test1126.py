from time import sleep

from fairino import Robot
from ctypes import sizeof
# A connection is established with the robot controller. A successful connection returns a robot object
# 与机器人控制器建立连接，连接成功返回一个机器人对象
robot = Robot.RPC('192.168.58.2')

import time



def serload(self):
    rtn,sdk = robot.GetSDKVersion()
    print(sdk)
    rtn = robot.SetLoadWeight(1,2)
    print(rtn)
    robot.CloseRPC()
    return

serload(robot)
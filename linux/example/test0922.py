from time import sleep
from fairino import Robot
from ctypes import sizeof
# A connection is established with the robot controller. A successful connection returns a robot object
# 与机器人控制器建立连接，连接成功返回一个机器人对象
robot = Robot.RPC('192.168.58.2')
import time


def TestKernelOTA(self):
    result = robot.KernelUpgrade("D://zUP/update_2025_head.img")
    print(f"KernelUpgrade result {result}")
    rtn,result = robot.GetKernelUpgradeResult()
    print(f"GetKernelUpgradeResult result {result}")
    robot.CloseRPC()

TestKernelOTA(robot)
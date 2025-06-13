from fairino import Robot
import time
# A connection is established with the robot controller. A successful connection returns a robot object
import time
import threading

# 与机器人控制器建立连接，连接成功返回一个机器人对象
robot = Robot.RPC('192.168.58.2')


def SmarttoolState(robot):
    while True:
        error,state = robot.GetSmarttoolBtnState()
        print(f"{state:016b}")  # 输出8位二进制，不足前面补0
        time.sleep(0.1)

SmarttoolState(robot)

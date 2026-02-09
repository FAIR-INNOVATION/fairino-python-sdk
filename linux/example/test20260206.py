from time import sleep

from fairino import Robot
from ctypes import sizeof
# A connection is established with the robot controller. A successful connection returns a robot object
# 与机器人控制器建立连接,连接成功返回一个机器人对象
robot = Robot.RPC('192.168.58.2')

import time


def TestDOReset(self):
    for i in range(16):
        robot.SetDO(i, 1, 0, 0)
        time.sleep(0.2)

    resetFlag = 0
    resumeReloadFlag = 0
    rtn = robot.SetOutputResetCtlBoxDO(resetFlag, resumeReloadFlag)
    robot.SetOutputResetCtlBoxAO(resetFlag, resumeReloadFlag)
    robot.SetOutputResetAxleDO(resetFlag, resumeReloadFlag)
    robot.SetOutputResetAxleAO(resetFlag, resumeReloadFlag)
    robot.SetOutputResetExtDO(resetFlag, resumeReloadFlag)
    robot.SetOutputResetExtAO(resetFlag, resumeReloadFlag)
    robot.SetOutputResetSmartToolDO(resetFlag, resumeReloadFlag)

    robot.ProgramLoad("/fruser/test.lua")
    robot.ProgramRun()

    time.sleep(2)
    robot.PauseMotion()
    time.sleep(2)
    robot.ResumeMotion()
    time.sleep(2)

    robot.CloseRPC()
    return 0

TestDOReset(robot)
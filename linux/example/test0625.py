from fairino import Robot
import time
# A connection is established with the robot controller. A successful connection returns a robot object
import time
import threading

# 与机器人控制器建立连接，连接成功返回一个机器人对象
robot = Robot.RPC('192.168.58.2')

import time

def TestWideVoltageCtrlBoxtemp(self):
    robot.SetWideBoxTempFanMonitorParam(1, 2)
    error, enable, period = robot.GetWideBoxTempFanMonitorParam()
    print(f"GetWideBoxTempFanMonitorParam enable is:{enable},period is:{period}")

    for i in range(100):
        error, pkg = robot.GetRobotRealTimeState()
        print(f"robot ctrl box temp is:{pkg.wideVoltageCtrlBoxTemp},fan current is:{pkg.wideVoltageCtrlBoxFanCurrent}")
        time.sleep(0.1)

    rtn = robot.SetWideBoxTempFanMonitorParam(0, 2)
    print(f"SetWideBoxTempFanMonitorParam rtn is:{rtn}")
    error, enable, period = robot.GetWideBoxTempFanMonitorParam()
    print(f"GetWideBoxTempFanMonitorParam enable is:{enable},period is:{period}")

    for i in range(100):
        error, pkg = robot.GetRobotRealTimeState()
        print(f"robot ctrl box temp is:{pkg.wideVoltageCtrlBoxTemp},fan current is:{pkg.wideVoltageCtrlBoxFanCurrent}")
        time.sleep(0.1)


import time

import time


def TestServoJ(self):
    j = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # JointPos
    epos = [0.0, 0.0, 0.0, 0.0]  # ExaxisPos
    vel = 0.0
    acc = 0.0
    cmdT = 0.008
    filterT = 0.0
    gain = 0.0
    flag = 0
    count = 500
    dt = 0.1
    cmdID = 0
    ret,j = robot.GetActualJointPosDegree(flag)
    if ret == 0:
        robot.ServoMoveStart()
        while count > 0:
            robot.ServoJ(joint_pos=j,axisPos=epos,acc=acc,vel=vel,cmdT=cmdT,filterT=filterT,gain=gain,id=cmdID)
            j[0] += dt
            count -= 1
            time.sleep(cmdT)
        robot.ServoMoveEnd()
    else:
        print(f"GetActualJointPosDegree errcode:{ret}")


# TestWideVoltageCtrlBoxtemp(robot)
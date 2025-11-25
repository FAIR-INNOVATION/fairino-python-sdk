from time import sleep
from fairino import Robot
from ctypes import sizeof
# A connection is established with the robot controller. A successful connection returns a robot object
# 与机器人控制器建立连接，连接成功返回一个机器人对象
robot = Robot.RPC('192.168.58.2')
import time

def ServoJTWithSafety(self):
    robot.ResetAllError()
    time.sleep(0.5)
    torques = [0.0] * 6
    rtn, torques = robot.GetJointTorques(1)
    robot.ServoJTStart()
    robot.DragTeachSwitch(1)
    checkFlag = 3
    # jPowerLimit = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
    jPowerLimit = [10.0, 10.0, 10.0, 10.0, 10.0, 10.0]
    jVelLimit = [50, 50.0, 50.0, 50.0, 50.0, 50.0]
    # jVelLimit = [181,80,80,80,80,80]
    count = 800000

    error = 0
    while count > 0:
        torques[2] = torques[2] + 0.01
        error = robot.ServoJT(torques, 0.008, checkFlag, jPowerLimit, jVelLimit)
        print(f"ServoJT rtn is {error}")
        count = count - 1
        time.sleep(0.001)

        rtn,pkg = robot.GetRobotRealTimeState()
        print(f"maincode {pkg.main_code},subcode {pkg.sub_code}")

    robot.DragTeachSwitch(0)
    error = robot.ServoJTEnd()
    return 0

def test(self):
    robot.Mode(0)
    time.sleep(1)
    rtn,state = robot.GetRobotRealTimeState()
    print(state.robot_mode)


# ServoJTWithSafety(robot)
# test(robot)
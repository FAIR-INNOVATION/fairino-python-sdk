from time import sleep
from fairino import Robot
# A connection is established with the robot controller. A successful connection returns a robot object
# 与机器人控制器建立连接，连接成功返回一个机器人对象
robot = Robot.RPC('192.168.58.2')
import time

def TestDragSwitchDetect(self):
    # Set torque detection and drag teach switches
    rtn = robot.SetTorqueDetectionSwitch(1)
    print(f"SetTorqueDetectionSwitch rtn: {rtn}")

    rtn = robot.DragTeachSwitch(1)
    print(f"DragTeachSwitch in rtn: {rtn}")

    time.sleep(1)

    rtn = robot.DragTeachSwitch(0)
    print(f"DragTeachSwitch out rtn: {rtn}")

    # Continuous error monitoring loop
    while True:
        rtn, [maincode, subcode] = robot.GetRobotErrorCode()
        print(f"robot maincode is {maincode}; subcode is {subcode}")

        time.sleep(1)

    robot.CloseRPC()
    return 0

TestDragSwitchDetect(robot)
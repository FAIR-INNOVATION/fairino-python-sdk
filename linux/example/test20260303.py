from time import sleep

from fairino import Robot
from ctypes import sizeof
# A connection is established with the robot controller. A successful connection returns a robot object
# 与机器人控制器建立连接,连接成功返回一个机器人对象
robot = Robot.RPC('192.168.58.2')

import time


def TestTPD(self):
    type = 1
    name = "tpd2025"
    period_ms = 4
    di_choose = 0
    do_choose = 0

    robot.SetTPDParam(type=type, name=name, period_ms=period_ms, di_choose=di_choose, do_choose=do_choose)

    robot.Mode(1)
    time.sleep(1)
    robot.DragTeachSwitch(1)
    robot.SetTPDStart(type=type, name=name, period_ms=period_ms, di_choose=di_choose, do_choose=do_choose)
    time.sleep(3)
    robot.SetWebTPDStop()
    robot.DragTeachSwitch(0)

    time.sleep(1)
    ovl = 100.0
    blend = 0
    start_pose = [0.0] * 6
    rtn = robot.LoadTPD(name)
    print(f"LoadTPD rtn is: {rtn}")

    rtn, start_pose = robot.GetTPDStartPose(name)
    print(f"start pose, xyz is: {start_pose[0]},{start_pose[1]},{start_pose[2]}. rpy is: {start_pose[3]},{start_pose[4]},{start_pose[5]}")
    # robot.MoveCart(desc_pos=start_pose, tool=0, user=0, vel=100, acc=100, ovl=ovl, blendT=-1, config=-1)
    #time.sleep(1)

    rtn = robot.MoveToTPDStart(name, 0, 100)
    print(f"MoveToTPDStart rtn is: {rtn}")

    rtn = robot.MoveTPD(name, blend, ovl)
    print(f"MoveTPD rtn is: {rtn}")
    time.sleep(5)

    robot.SetTPDDelete(name)

    robot.CloseRPC()
    return 0

TestTPD(robot)
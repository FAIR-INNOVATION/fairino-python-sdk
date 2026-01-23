from time import sleep

from fairino import Robot
from ctypes import sizeof
# A connection is established with the robot controller. A successful connection returns a robot object
# 与机器人控制器建立连接,连接成功返回一个机器人对象
robot = Robot.RPC('192.168.58.2')

import time


def move(self):
    joint_pos1 = [38.800, -77.317, 89.175, -96.936, -94.134, -49.181]
    joint_pos2 = [42.898, -77.317, 89.175, -96.936, -94.134, -49.181]
    desc_pos1 = [644.979, -71.290, 519.920, 28.406, 7.282, 65.968]
    desc_pos2 = [630.934, -109.794, 510.026, 27.831, 9.601, 62.554]
    desc_pos3 = [409.214,-359.354,528.239,22.703,21.534,37.347]

    # joint_pos1 = [69.625,-105.826,111.604,-91.064,-90.654,-49.575]
    # desc_pos1 = [426.319,-362.973,572.615,22.698,21.534,37.347]
    # joint_pos2 = [69.625,-106.092,107.438,-86.629,-90.654,-49.575]
    # desc_pos2 = [409.214,-359.354,548.239,22.703,21.534,37.347]


    # joint_pos1 = [69.625,-105.826,111.604,-91.059,-90.654,-49.575]
    # desc_pos1 = [426.317,-362.974,572.619,22.702,21.534,37.347]
    # joint_pos2 = [41.834,-101.106,111.615,-100.359,-93.059,-49.619]
    # desc_pos2 = [504.495,-158.799,568.615,32.152,11.594,64.809]

    epos1 = [0.000, 0.000, 0.000, 0.000]
    epos2 = [-20.0, 0.000, 0.000, 0.000]

    offset_pos = [0.0] * 6
    tool = 1
    user = 1
    vel = 100
    acc = 100
    ovl = 30
    blendT = -1

    rtn = robot.ExtAxisSyncMoveL(joint_pos=joint_pos1,desc_pos=desc_pos1, tool=tool, user=user, vel=vel, acc=acc, ovl=ovl, blendR=-1, exaxis_pos=epos1, offset_flag=0, offset_pos=offset_pos)
    print(f"ExtAxisSyncMoveL rtn is {rtn}")
    rtn = robot.ExtAxisSyncMoveL(joint_pos=joint_pos2,desc_pos=desc_pos2, tool=tool, user=user, vel=vel, acc=acc, ovl=ovl, blendR=-1, exaxis_pos=epos2, offset_flag=0, offset_pos=offset_pos)
    print(f"ExtAxisSyncMoveL rtn is {rtn}")

    # pos=[0.000, 0.000, 0.000, 0.000]
    # ovl=40
    # rtn = robot.ExtAxisMove(pos,ovl)
    print(f"ExtAxisMove rtn is {rtn}")
    robot.CloseRPC()
    # time.sleep(9999)  # 9999999毫秒 ≈ 9999.999秒
    return 0

move(robot)
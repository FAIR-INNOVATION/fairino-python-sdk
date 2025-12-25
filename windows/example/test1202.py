from time import sleep

from fairino import Robot
from ctypes import sizeof
# A connection is established with the robot controller. A successful connection returns a robot object
# 与机器人控制器建立连接，连接成功返回一个机器人对象
robot = Robot.RPC('192.168.58.2')

import time


def TestFTControlWithAdjustCoeff(self):
    sensor_id = 10
    select = [0, 0, 1, 0, 0, 0]
    ft_pid = [0.0008, 0.0, 0.0, 0.0, 0.0, 0.0]
    adj_sign = 0
    ILC_sign = 0
    max_dis = 1000.0
    max_ang = 20.0

    ft = [0.0] * 6  # [fx, fy, fz, tx, ty, tz]
    epos = [0.0] * 4

    j1 = [80.765, -98.795, 106.548, -97.734, -89.999, 94.842]
    j2 = [43.067, -84.429, 92.620, -98.175, -90.011, 57.144]
    desc_p1 = [5.009, -547.463, 262.053, -179.999, -0.019, 75.923]
    desc_p2 = [-347.966, -547.463, 262.048, -180.000, -0.019, 75.923]
    offset_pos = [0.0] * 6

    M = [2.0, 2.0]
    B = [15.0, 15.0]
    threshold = [1.0, 1.0]
    adjustCoeff = [1.0, 0.8]
    polishRadio = 0.0
    filter_Sign = 0
    posAdapt_sign = 1
    isNoBlock = 0

    ft[2] = -10.0  # fz = -10.0

    while True:
        rtn = robot.FT_Control(1, sensor_id, select, ft, ft_pid, adj_sign, ILC_sign, max_dis, max_ang, M, B, threshold,
                               adjustCoeff, 0, 0, 1, 0)
        print(f"FT_Control start rtn is {rtn}")

        rtn = robot.MoveL(desc_pos=desc_p1, tool=1, user=0, vel=100, acc=100, ovl=100, blendR=-1, blendMode = 0, exaxis_pos=epos, search=0, offset_flag=0, offset_pos=offset_pos)
        rtn = robot.MoveL(desc_pos=desc_p2, tool=1, user=0, vel=100, acc=100, ovl=100, blendR=-1, blendMode = 0, exaxis_pos=epos, search=0, offset_flag=0, offset_pos=offset_pos)

        rtn = robot.FT_Control(0, sensor_id, select, ft, ft_pid, adj_sign, ILC_sign, max_dis, max_ang, M, B, threshold,
                               adjustCoeff, 0, 0, 1, 0)
        print(f"FT_Control end rtn is {rtn}")

    robot.CloseRPC()
    return 0

def move(self):
    sensor_id = 10
    select = [0, 0, 1, 0, 0, 0]
    ft_pid = [0.0008, 0.0, 0.0, 0.0, 0.0, 0.0]
    adj_sign = 0
    ILC_sign = 0
    max_dis = 1000.0
    max_ang = 20.0

    ft = [0.0] * 6  # [fx, fy, fz, tx, ty, tz]
    epos = [0.0] * 4

    j1 = [80.765, -98.795, 106.548, -97.734, -89.999, 94.842]
    j2 = [43.067, -84.429, 92.620, -98.175, -90.011, 57.144]
    desc_p1 = [ -321.428,-417.448,488.218,-178.667,1.797,-169.984]
    desc_p2 = [-430.053,74.317,416.062,171.196,3.319,127.589]


    M = [2.0, 2.0]
    B = [15.0, 15.0]
    threshold = [1.0, 1.0]
    adjustCoeff = [1.0, 0.8]
    polishRadio = 0.0
    filter_Sign = 0
    posAdapt_sign = 1
    isNoBlock = 0

    while(True):

        print("======================================================================================")
        rtn = robot.MoveL(desc_pos=desc_p1, tool=0, user=0, vel=100, acc=100, ovl=100, blendR=-1.0, blendMode=0,
                          exaxis_pos=epos, search=0, offset_flag=0)
        # print(rtn)
        rtn,pos = robot.GetActualTCPPose()
        print(pos)
        sleep(1)
        rtn,pos = robot.GetActualTCPPose()
        print(pos)

        sleep(1)
        print("======================================================================================")
        rtn = robot.MoveCart(desc_pos=desc_p2, tool=0, user=0, vel=100, acc=100, ovl=100, blendT=-1.0)
        # print(rtn)
        rtn, pos = robot.GetActualTCPPose()
        print(pos)
        sleep(1)
        rtn, pos = robot.GetActualTCPPose()
        print(pos)


    robot.CloseRPC()
    return 0

# TestFTControlWithAdjustCoeff(robot)
move(robot)

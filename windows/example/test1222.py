from time import sleep

from fairino import Robot
from ctypes import sizeof
# A connection is established with the robot controller. A successful connection returns a robot object
# 与机器人控制器建立连接，连接成功返回一个机器人对象
robot = Robot.RPC('192.168.58.2')

import time


def move(self):
    j1 = [-11.904, -99.669, 117.473, -108.616, -91.726, 74.256]
    j2 = [-45.615, -106.172, 124.296, -107.151, -91.282, 74.255]
    j3 = [-29.777, -84.536, 109.275, -114.075, -86.655, 74.257]
    j4 = [-31.154, -95.317, 94.276, -88.079, -89.740, 74.256]
    desc_pos1 = [-419.524, -13.000, 351.569, -178.118, 0.314, 3.833]
    desc_pos2 = [-321.222, 185.189, 335.520, -179.030, -1.284, -29.869]
    desc_pos3 = [-487.434, 154.362, 308.576, 176.600, 0.268, -14.061]
    desc_pos4 = [-443.165, 147.881, 480.951, 179.511, -0.775, -15.409]
    offset_pos = [0.0] * 6
    epos = [0.0] * 4

    tool = 0
    user = 0
    vel = 100.0
    acc = 100.0
    ovl = 100.0
    oacc = 100.0
    blendT = 0.0
    blendR = 0.0
    flag = 0
    search = 0
    blendMode = 0
    velAccMode = 0

    robot.SetSpeed(20)

    rtn = robot.MoveJ(joint_pos=j1, tool=tool, user=user, vel=vel, acc=acc, ovl=ovl, exaxis_pos=epos, blendT=blendT, offset_flag=flag, offset_pos=offset_pos)
    print(f"movej errcode:{rtn}")

    rtn = robot.MoveL(desc_pos=desc_pos2, tool=tool, user=user, vel=vel, acc=acc, ovl=ovl, blendR=blendR, blendMode=blendMode, exaxis_pos=epos, search=search, offset_flag=flag, offset_pos=offset_pos,
                      oacc=oacc, velAccParamMode=velAccMode)
    print(f"movel errcode:{rtn}")

    rtn = robot.MoveC(desc_pos_p=desc_pos3, tool_p=tool, user_p=user, vel_p=vel, acc_p=acc, exaxis_pos_p=epos, offset_flag_p=flag, offset_pos_p=offset_pos, desc_pos_t=desc_pos4, tool_t=tool, user_t=user, vel_t=vel,
                      acc_t=acc, exaxis_pos_t=epos, offset_flag_t=flag, offset_pos_t=offset_pos, ovl=ovl, blendR=blendR, oacc=oacc, velAccParamMode=velAccMode)
    print(f"movec errcode:{rtn}")

    rtn = robot.MoveJ(joint_pos=j2, tool=tool, user=user, vel=vel, acc=acc, ovl=ovl, exaxis_pos=epos, blendT=blendT, offset_flag=flag, offset_pos=offset_pos)
    print(f"movej errcode:{rtn}")

    rtn = robot.Circle(desc_pos_p=desc_pos3, tool_p=tool, user_p=user, vel_p=vel, acc_p=acc, exaxis_pos_p=epos, desc_pos_t=desc_pos1, tool_t=tool, user_t=user, vel_t=vel, acc_t=acc, exaxis_pos_t=epos, ovl=ovl,
                       offset_flag=flag, offset_pos=offset_pos, oacc=oacc, blendR=-1, velAccParamMode=velAccMode)
    print(f"circle errcode:{rtn}")

    rtn = robot.MoveCart(desc_pos=desc_pos4, tool=tool, user=user, vel=vel, acc=acc,ovl=ovl, blendT=blendT, config=-1)
    print(f"MoveCart errcode:{rtn}")

    rtn = robot.MoveJ(joint_pos=j1, tool=tool, user=user, vel=vel, acc=acc, ovl=ovl, exaxis_pos=epos, blendT=blendT, offset_flag=flag, offset_pos=offset_pos)
    print(f"movej errcode:{rtn}")

    rtn = robot.MoveL(desc_pos=desc_pos2, tool=tool, user=user, vel=vel, acc=acc, ovl=ovl, blendR=blendR, blendMode=blendMode, exaxis_pos=epos, search=search, offset_flag=flag, offset_pos=offset_pos, config=-1,
                      velAccParamMode=velAccMode)
    print(f"movel errcode:{rtn}")

    rtn = robot.MoveC(desc_pos_p=desc_pos3, tool_p=tool, user_p=user, vel_p=vel, acc_p=acc, exaxis_pos_p=epos, offset_flag_p=flag, offset_pos_p=offset_pos, desc_pos_t=desc_pos4, tool_t=tool, user_t=user, vel_t=vel, acc_t=acc,
                      exaxis_pos_t=epos, offset_flag_t=flag, offset_pos_t=offset_pos, ovl=ovl, blendR=blendR, config=-1, velAccParamMode=velAccMode)
    print(f"movec errcode:{rtn}")

    rtn = robot.MoveJ(joint_pos=j2, tool=tool, user=user, vel=vel, acc=acc, ovl=ovl, exaxis_pos=epos, blendT=blendT, offset_flag=flag, offset_pos=offset_pos)
    print(f"movej errcode:{rtn}")

    rtn = robot.Circle(desc_pos_p=desc_pos3, tool_p=tool, user_p=user, vel_p=vel, acc_p=acc, exaxis_pos_p=epos, desc_pos_t=desc_pos1, tool_t=tool, user_t=user, vel_t=vel, acc_t=acc, exaxis_pos_t=epos, ovl=ovl, offset_flag=flag,
                       offset_pos=offset_pos, oacc=oacc, blendR=-1, velAccParamMode=velAccMode)
    print(f"circle errcode:{rtn}")

    robot.CloseRPC()
    return 0

move(robot)
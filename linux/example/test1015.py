from time import sleep
from fairino import Robot
from ctypes import sizeof
# A connection is established with the robot controller. A successful connection returns a robot object
# 与机器人控制器建立连接，连接成功返回一个机器人对象
robot = Robot.RPC('192.168.58.2')
import time

def TestServoJ(self):
    j = [0.0] * 6
    epos = [0.0] * 4

    vel = 0.0
    acc = 0.0
    cmdT = 0.008
    filterT = 0.0
    gain = 0.0
    flag = 0
    count = 1000
    dt = 0.1
    cmdID = 0
    ret, j = robot.GetActualJointPosDegree(flag)
    if ret == 0:
        cmdID += 1
        robot.ServoMoveStart()
        while count:
            robot.ServoJ(joint_pos=j,axisPos= epos,acc= acc,vel= vel, cmdT=cmdT, filterT=filterT, gain=gain, id=cmdID)
            j[0] -= dt
            count -= 1
            time.sleep(cmdT)
            rtn,pkg = robot.GetRobotRealTimeState()
            print(f"Servoj Count {pkg.servoJCmdNum}; last pos is {pkg.lastServoTarget[0]},{pkg.lastServoTarget[1]},{pkg.lastServoTarget[2]},{pkg.lastServoTarget[3]},{pkg.lastServoTarget[4]},{pkg.lastServoTarget[5]}")

            if count < 50:
                robot.MotionQueueClear()
                print(f"After queue clear, Servoj Count {pkg.servoJCmdNum}; last pos is {pkg.lastServoTarget[0]},{pkg.lastServoTarget[1]},{pkg.lastServoTarget[2]},{pkg.lastServoTarget[3]},{pkg.lastServoTarget[4]},{pkg.lastServoTarget[5]}")
                break
        robot.ServoMoveEnd()
    else:
        print(f"GetActualJointPosDegree errcode:{ret}")

    robot.CloseRPC()
    return 0

def TestSlavePortErr(self):
    inRecvErr = [0] * 8
    inCRCErr = [0] * 8
    inTransmitErr = [0] * 8
    inLinkErr = [0] * 8
    outRecvErr = [0] * 8
    outCRCErr = [0] * 8
    outTransmitErr = [0] * 8
    outLinkErr = [0] * 8

    rtn,inRecvErr, inCRCErr, inTransmitErr, inLinkErr, outRecvErr, outCRCErr, outTransmitErr, outLinkErr = robot.GetSlavePortErrCounter()

    for i in range(8):
        if inRecvErr[i] != 0:
            print(f"inRecvErr {i} is {inRecvErr[i]}")

        if inCRCErr[i] != 0:
            print(f"inCRCErr {i} is {inCRCErr[i]}")

        if inTransmitErr[i] != 0:
            print(f"inTransmitErr {i} is {inTransmitErr[i]}")

        if inLinkErr[i] != 0:
            print(f"inLinkErr {i} is {inLinkErr[i]}")

        if outRecvErr[i] != 0:
            print(f"outRecvErr {i} is {outRecvErr[i]}")

        if outCRCErr[i] != 0:
            print(f"outCRCErr {i} is {outCRCErr[i]}")

        if outTransmitErr[i] != 0:
            print(f"outTransmitErr {i} is {outTransmitErr[i]}")

        if outLinkErr[i] != 0:
            print(f"outLinkErr {i} is {outLinkErr[i]}")

    print("others has no err!")

    for i in range(8):
        robot.SlavePortErrCounterClear(i)

    robot.CloseRPC()
    return 0


def TestVelFeedForwardRatio(self):
    setRadio = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
    robot.SetVelFeedForwardRatio(setRadio)

    getRadio = [0.0] * 6
    rtn,getRadio = robot.GetVelFeedForwardRatio()
    print(f"{getRadio[0]},{getRadio[1]},{getRadio[2]},{getRadio[3]},{getRadio[4]},{getRadio[5]}")

    robot.CloseRPC()


def TestSpiral(self):
    j = [67.957, -81.482, 87.595, -95.691, -94.899, -9.727]
    desc_pos = [-123.142, -551.735, 430.549, 178.753, -4.757, 167.754]
    offset_pos1 = [50.0, 0.0, 0.0, -30.0, 0.0, 0.0]
    offset_pos2 = [50.0, 0.0, 0.0, -30.0, 0.0, 0.0]
    epos = [0.0] * 4

    # 螺旋参数使用列表或字典表示
    sp = [2, 30.0, 50.0, 10.0, 10.0, 0, 1]  # [circle_num, circle_angle, rad_init, rad_add, rotaxis_add, rot_direction, velAccMode]

    tool = 0
    user = 0
    vel = 30.0
    acc = 60.0
    ovl = 100.0
    blendT = -1.0
    flag = 2

    robot.SetSpeed(20)

    rtn = robot.MoveJ(joint_pos=j, tool=tool, user=user, vel=vel, acc=acc, ovl=ovl, exaxis_pos=epos, blendT=blendT, offset_flag=flag, offset_pos=offset_pos1)
    print(f"movej errcode:{rtn}")

    rtn = robot.NewSpiral(desc_pos=desc_pos, tool=tool, user=user, vel=vel, acc=acc, exaxis_pos=epos, ovl=ovl, offset_flag=flag, offset_pos=offset_pos2, param=sp)
    print(f"newspiral errcode:{rtn}")

    robot.CloseRPC()
    return 0

def TestFTControlWithDamping(self):
    sensor_id = 10
    select = [0, 0, 1, 0, 0, 0]
    ft_pid = [0.0008, 0.0, 0.0, 0.0, 0.0, 0.0]
    adj_sign = 0
    ILC_sign = 0
    max_dis = 100.0
    max_ang = 20.0

    ft = [0.0] * 6  # [fx, fy, fz, tx, ty, tz]
    ft[2] = -10.0  # fz = -10.0
    epos = [0.0] * 4
    j1 = [-118.985, -86.882, -118.139, -65.019, 90.002, 54.951]
    j2 = [-77.055, -77.218, -126.219, -66.591, 90.028, 96.881]
    desc_p1 = [-300.856, -332.618, 309.240, 179.976, -0.031, 96.065]
    desc_p2 = [-16.399, -383.760, 309.312, 179.975, -0.031, 96.064]
    offset_pos = [0.0] * 6

    M = [2.0, 2.0]
    B = [8.0, 8.0]
    polishRadio = 0.0
    filter_Sign = 0
    posAdapt_sign = 0
    isNoBlock = 0

    ftCoord = [0.0] * 6
    robot.FT_SetRCS(2, ftCoord)
    rtn = robot.FT_Control(1, sensor_id, select, ft, ft_pid, adj_sign, ILC_sign, max_dis, max_ang, M, B, 0, 0, 1, 0)
    print(f"FT_Control start rtn is {rtn}")
    rtn = robot.MoveL(desc_pos=desc_p1, tool=0, user=0, vel=100.0, acc=100.0, ovl=20.0, blendR=-1.0, exaxis_pos=epos, search=0, offset_flag=0, offset_pos=offset_pos)
    rtn = robot.MoveL(desc_pos=desc_p2, tool=0, user=0, vel=100.0, acc=100.0, ovl=20.0, blendR=-1.0, exaxis_pos=epos, search=0, offset_flag=0, offset_pos=offset_pos)
    rtn = robot.FT_Control(1, sensor_id, select, ft, ft_pid, adj_sign, ILC_sign, max_dis, max_ang, M, B, 0, 0, 1, 0)
    print(f"FT_Control end rtn is {rtn}")

    robot.CloseRPC()
    return 0

TestServoJ(robot)
# TestSlavePortErr(robot)
# TestVelFeedForwardRatio(robot)
# TestSpiral(robot)
# TestFTControlWithDamping(robot)




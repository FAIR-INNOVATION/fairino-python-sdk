from time import sleep
from fairino import Robot
from ctypes import sizeof
# A connection is established with the robot controller. A successful connection returns a robot object
# 与机器人控制器建立连接，连接成功返回一个机器人对象
robot = Robot.RPC('192.168.58.2')

import time


def testAxleGenCom(self):

    led_on = [0xAB, 0xBA, 0x12, 0x01, 0x01, 0x79]
    led_off = [0xAB, 0xBA, 0x12, 0x01, 0x00, 0x78]
    version = [0xAB, 0xBA, 0x11, 0x00, 0x76]
    state = [0xAB, 0xBA, 0x1B, 0x01, 0xAA, 0x2B]
    cycleState = [0xAB, 0xBA, 0x12, 0x01, 0x00, 0x78]
    cnt = 1

    p1Joint = [88.708, -86.178, 140.989, -141.825, -89.162, -49.879]
    p1Desc = [188.007, -377.850, 260.207, 178.715, 2.823, -131.466]
    p2Joint = [112.131, -75.554, 126.989, -139.027, -88.044, -26.477]
    p2Desc = [368.003, -377.848, 260.211, 178.715, 2.823, -131.465]

    exaxisPos = [0, 0, 0, 0]
    offdese = [0, 0, 0, 0, 0, 0]

    #开启末端透传功能
    robot.SetAxleGenComEnable(1)
    robot.SetAxleLuaEnable(1)

    while cnt <= 10000:
        #读取版本号
        ret,rcvdata = robot.SndRcvAxleGenComCmdData(len_snd=5, sndBuff=version, len_rcv=10)
        print(ret)
        print(rcvdata)
        print(f"hard version : {rcvdata[4]},hard code:{rcvdata[5]}, soft version:{rcvdata[6]} {rcvdata[7]}, soft code:{rcvdata[8]}")
        if ret != 0:
            break
        time.sleep(1)
        # 读取艾灸头在位状态
        ret,rcvdata = robot.SndRcvAxleGenComCmdData(6, state, 6)
        print(f"state : {rcvdata[4]} ")
        time.sleep(1)
        # 开启艾灸头激光
        ret,rcvdata = robot.SndRcvAxleGenComCmdData(6, led_on, 6)
        print(f"led on rcv data is: {rcvdata[0]}, {rcvdata[1]}, {rcvdata[2]}, {rcvdata[3]}, {rcvdata[4]}, {rcvdata[5]}")
        robot.MoveJ(joint_pos=p1Joint, tool=0, user=0, vel=100, acc=100, ovl=100, exaxis_pos=exaxisPos, blendT=-1,
                          offset_flag=0, offset_pos=offdese)
        time.sleep(4)
        # 关闭艾灸头激光
        ret, rcvdata = robot.SndRcvAxleGenComCmdData(6, led_off, 6)
        print(f"led off rcv data is: {rcvdata[0]}, {rcvdata[1]}, {rcvdata[2]}, {rcvdata[3]}, {rcvdata[4]}, {rcvdata[5]}")
        robot.MoveJ(joint_pos=p2Joint, tool=0, user=0, vel=100, acc=100, ovl=100, exaxis_pos=exaxisPos, blendT=-1,
                    offset_flag=0, offset_pos=offdese)
        time.sleep(1)
        print(f"***********************complate No. {cnt} SDK test*****************************")
        cnt = cnt + 1

    robot.CloseRPC()
    return 0

testAxleGenCom(robot)

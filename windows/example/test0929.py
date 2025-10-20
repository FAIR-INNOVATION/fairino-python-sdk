from time import sleep
from fairino import Robot
from ctypes import sizeof
# A connection is established with the robot controller. A successful connection returns a robot object
# 与机器人控制器建立连接，连接成功返回一个机器人对象
robot = Robot.RPC('192.168.58.2')
import time

import time


def testLaserRecordAndReplayMoveC(self):
    cnt = 1
    while cnt < 2:
        # 运动到扫描的起点
        startjointPos1 = [-15.647, -119.042, 109.960, -71.222, -73.948, -122.687]
        startdescPose1 = [-274.669, -122.896, 246.972, -161.315, -0.312, -164.381]
        exaxisPos = [0, 0, 0, 0]
        offdese = [0, 0, 0, 0, 0, 0]

        robot.MoveJ(joint_pos=startjointPos1, tool=1, user=0, vel=100, acc=100, ovl=50, exaxis_pos=exaxisPos, blendT=-1, offset_flag=0, offset_pos=offdese)

        # 运动到扫描的起点
        startjointPos = [-23.965, -150.841, 137.707, -89.747, -56.114, -122.685]
        startdescPose = [-274.002, -189.344, 194.938, -157.388, -28.759, -173.209]
        rtn = robot.MoveL(desc_pos=startdescPose, tool=1, user=0, vel=10)
        print(rtn)

        # 开始轨迹记录
        robot.LaserSensorRecord1(2, 10)

        # 运动到需要记录的终点
        midjointPos = [36.350, -59.819, 63.114, -51.373, -105.011, 98.495]
        middescPose = [-370.608, -294.229, 181.531, -158.073, -39.221, 25.737]
        print("111111")

        endjointPos = [-26.944, -101.993, 115.794, -72.164, -53.080, 164.700]
        enddescPose = [-353.625, -155.023, 185.415, -151.407, -39.177, -122.813]

        # robot.Circle(joint_pos_p= midjointPos, desc_pos_p=middescPose, tool_p=1, user_p=0, vel_p=10, acc_p=100, exaxis_pos_p=exaxisPos,joint_pos_t= endjointPos, desc_pos_t=enddescPose, tool_t=1, user_t=0, vel_t=10, acc_t=100,
        #              exaxis_pos_t=exaxisPos, ovl=100, offset_flag=0, offset_pos=offdese, oacc=100, blendR=-1, velAccParamMode=0)
        robot.MoveC(joint_pos_p=midjointPos, desc_pos_p=middescPose, tool_p=1, user_p=0, vel_p=10, acc_p=100,
                     exaxis_pos_p=exaxisPos, joint_pos_t=endjointPos, desc_pos_t=enddescPose, tool_t=1, user_t=0,
                     vel_t=10, acc_t=100)
        print("222222")

        # 停止记录
        robot.LaserSensorRecord1(0, 10)
        print("333333")

        time.sleep(2)
        # robot.StopMotion()

        startjointPos2 = [-6.592, -140.898, 122.764, -88.529, -81.143, -82.069]
        startdescPose2 = [-251.875, -124.247, 250.719, -168.899, -15.289, 165.278]
        robot.MoveJ(joint_pos=startjointPos2, tool=1, user=0, vel=60, acc=100, ovl=50, exaxis_pos=exaxisPos, blendT=-1, offset_flag=0, offset_pos=offdese)
        print("4444444")

        # 运动到记录的焊缝起点
        robot.MoveToLaserRecordStart(1, 30)
        # 开始轨迹复现
        robot.LaserSensorReplay(10, 100)

        robot.MoveLTR()
        # 停止轨迹复现
        robot.LaserSensorRecord1(0, 10)
        cnt += 1

    robot.CloseRPC()

testLaserRecordAndReplayMoveC(robot)
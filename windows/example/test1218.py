from time import sleep

from fairino import Robot
from ctypes import sizeof
# A connection is established with the robot controller. A successful connection returns a robot object
# 与机器人控制器建立连接，连接成功返回一个机器人对象
robot = Robot.RPC('192.168.58.2')

import time


def move(self):
    startdescPose = [-252.898, -611.453, 211.692, 178.053, -5.329, 114.255]
    startjointPos = [59.127, -59.093, 88.934, -115.191, -93.252, 35.095]
    enddescPose = [-659.192, -188.885, 211.698, 178.054, -5.329, 114.255]
    endjointPos = [8.238, -56.580, 86.031, -119.046, -95.657, -15.906]

    midarcdescPose = [-348.296, -611.453, 211.698, 178.053, -5.329, 114.255]
    midarcjointPos = [52.509, -54.057, 80.630, -112.327, -93.766, 28.484]
    endarcdescPose = [-348.297, -478.796, 211.696, 178.053, -5.329, 114.255]
    endarcjointPos = [44.748, -67.215, 102.017, -121.104, -94.304, 20.722]

    exaxisPos = [0.0] * 4
    offdese = [0.0] * 6

    robot.MoveL(desc_pos=startdescPose,tool=1,user=0,vel=100,acc=100,ovl=100,blendR=-1,blendMode=0,exaxis_pos=exaxisPos,search=0,offset_flag=0,offset_pos=offdese,oacc=200.0,velAccParamMode=1)
    robot.MoveL(desc_pos=enddescPose,tool=1,user=0,vel=100,acc=100,ovl=100,blendR=-1,blendMode=0,exaxis_pos=exaxisPos,search=0,offset_flag=0,offset_pos=offdese,oacc=200.0,velAccParamMode=1)

    robot.MoveL(desc_pos=startdescPose,tool=1,user=0,vel=100,acc=100,ovl=100,blendR=-1,blendMode=0,exaxis_pos=exaxisPos,search=0,offset_flag=0,offset_pos=offdese,oacc=200.0,velAccParamMode=1)
    robot.MoveC(desc_pos_p=midarcdescPose,tool_p=1,user_p=0,vel_p=100,acc_p=100,exaxis_pos_p=exaxisPos,offset_flag_p=0,offset_pos_p=offdese, desc_pos_t=endarcdescPose,
                tool_t=1,user_t=0,vel_t=100,acc_t=100,exaxis_pos_t=exaxisPos,offset_flag_t=0,offset_pos_t=offdese,ovl=100,blendR=-1,oacc=200,velAccParamMode=1)

    robot.MoveL(desc_pos=startdescPose,tool=1,user=0,vel=100,acc=100,ovl=100,blendR=-1,blendMode=0,exaxis_pos=exaxisPos,search=0,offset_flag=0,offset_pos=offdese,oacc=200.0,velAccParamMode=1)
    rtn = robot.Circle(desc_pos_p=midarcdescPose,tool_p=1,user_p=0,vel_p=100,acc_p=100,exaxis_pos_p=exaxisPos,desc_pos_t=endarcdescPose,tool_t=1,user_t=0,
                       vel_t=100,acc_t=100,exaxis_pos_t=exaxisPos,ovl=100,offset_flag=-1,offset_pos=offdese,oacc=210,blendR=-1,velAccParamMode=1)
    print(f"Circle rtn is {rtn}")
    time.sleep(100)  # 100秒延时

    robot.CloseRPC()
    return 0
def test(self):
    # while True:
    #     rtn,pos = robot.GetActualTCPPose()
    #     print(pos[2])
    #     time.sleep(0.008)
    rtn = robot.ServoCart(mode=1, desc_pos=[1.0,0.0,0.0,0.0,0.0,0.0])
    print(rtn)

# move(robot)
test(robot)
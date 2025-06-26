from fairino import Robot
import time
# A connection is established with the robot controller. A successful connection returns a robot object
import time
import threading

# 与机器人控制器建立连接，连接成功返回一个机器人对象
robot = Robot.RPC('192.168.58.2')

import time


def DragControl(self):
    M = [15.0, 15.0, 15.0, 0.5, 0.5, 0.1]
    B = [150.0, 150.0, 150.0, 5.0, 5.0, 1.0]
    K = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    F = [10.0, 10.0, 10.0, 1.0, 1.0, 1.0]

    rtn = robot.EndForceDragControl(status=1,asaptiveFlag= 0,interfereDragFlag= 0,ingularityConstraintsFlag= 0,forceCollisionFlag= 1,M= M,B= B,K= K,F= F,Fmax= 50,Vmax=100)
    print(f"force drag control start rtn is:{rtn}")
    time.sleep(5)

    rtn = robot.EndForceDragControl(status=0, asaptiveFlag=0, interfereDragFlag=0, ingularityConstraintsFlag=0,forceCollisionFlag=1, M=M, B=B, K=K, F=F, Fmax=50, Vmax=100)
    print(f"force drag control end rtn is:{rtn}")

    rtn = robot.ResetAllError()
    print(f"ResetAllError rtn is:{rtn}")

    rtn = robot.EndForceDragControl(status=1, asaptiveFlag=0, interfereDragFlag=0, ingularityConstraintsFlag=0,forceCollisionFlag=1, M=M, B=B, K=K, F=F, Fmax=50, Vmax=100)
    print(f"force drag control start again rtn is:{rtn}")
    time.sleep(5)

    rtn = robot.EndForceDragControl(status=0, asaptiveFlag=0, interfereDragFlag=0, ingularityConstraintsFlag=0,forceCollisionFlag=1, M=M, B=B, K=K, F=F, Fmax=50, Vmax=100)
    print(f"force drag control end again rtn is:{rtn}")

def TestBlend(self):
    exaxisPos = [0, 0, 0, 0]
    offdese = [0, 0, 0, 0, 0, 0]

    # 第一组关节位置和描述位姿
    JP1 = [55.203, -69.138, 75.617, -103.969, -83.549, -0.001]
    DP1 = [0]*6
    JP2 = [57.646, -61.846, 59.286, -69.645, -99.735, 3.824]
    DP2 = [0]*6
    JP3 = [57.304, -61.380, 58.260, -67.641, -97.447, 2.685]
    DP3 = [0]*6
    JP4 = [57.297, -61.373, 58.250, -67.637, -97.448, 2.677]
    DP4 = [0]*6
    JP5 = [23.845, -108.202, 111.300, -80.971, -106.753, -30.246]
    DP5 = [0]*6
    JP6 = [23.845, -108.202, 111.300, -80.971, -106.753, -30.246]
    DP6 = [0]*6

    # 正运动学计算
    error,DP1 = robot.GetForwardKin(JP1)
    error,DP2 = robot.GetForwardKin(JP2)
    error,DP3 = robot.GetForwardKin(JP3)
    error,DP4 = robot.GetForwardKin(JP4)
    error,DP5 = robot.GetForwardKin(JP5)
    error,DP6 = robot.GetForwardKin(JP6)

    # 运动指令
    robot.MoveJ(joint_pos=JP1,tool=0,user= 0,vel= 100,blenT= -1)
    robot.MoveJ(joint_pos=JP2, tool=0, user=0, vel=100, blenT=200)
    robot.MoveJ(joint_pos=JP3, tool=0, user=0, vel=100, blenT=200)
    robot.MoveJ(joint_pos=JP4, tool=0, user=0, vel=100, blenT=200)
    robot.MoveJ(joint_pos=JP5, tool=0, user=0, vel=100, blenT=200)
    robot.MoveJ(joint_pos=JP6, tool=0, user=0, vel=100, blenT=200)

    # 第二组关节位置和描述位姿
    JP7 = [-10.503, -93.654, 111.333, -84.702, -103.479, -30.179]
    DP7 = [0]*6
    JP8 = [-10.503, -93.654, 111.333, -84.702, -103.479, -30.179]
    DP8 = [0]*6
    JP9 = [-10.503, -93.654, 111.333, -84.702, -103.479, -30.179]
    DP9 = [0]*6
    JP10 = [-30.623, -74.158, 89.844, -91.942, -97.060, -30.180]
    DP10 = [0]*6
    JP11 = [-34.797, -72.641, 93.917, -104.961, -84.449, -30.287]
    DP11 = [0]*6
    JP12 = [-17.454, -58.309, 82.054, -111.034, -109.900, -30.241]
    DP12 = [0]*6
    JP13 = [-4.930, -72.469, 100.631, -109.906, -76.760, -10.947]
    DP13 = [0]*6

    # 正运动学计算
    error,DP7 = robot.GetForwardKin(JP7)
    error,DP8 = robot.GetForwardKin(JP8)
    error,DP9 = robot.GetForwardKin(JP9)
    error,DP10 = robot.GetForwardKin(JP10)
    error,DP11 = robot.GetForwardKin(JP11)
    error,DP12 = robot.GetForwardKin(JP12)
    error,DP13 = robot.GetForwardKin(JP13)

    # 运动指令
    robot.MoveJ(joint_pos=JP7, tool=0, user=0, vel=100, blenT=200)
    robot.MoveL(desc_pos=DP8,tool=0,user=0,vel=100,blendR=20)
    robot.MoveJ(joint_pos=JP9, tool=0, user=0, vel=100, blenT=200)
    robot.MoveL(desc_pos=DP10, tool=0, user=0, vel=100, blendR=20)
    robot.MoveJ(joint_pos=JP11, tool=0, user=0, vel=100, blenT=200)
    robot.MoveC(desc_pos_p=DP12,tool_p=0,user_p=0,desc_pos_t=DP13,tool_t=0,user_t=0,vel_p=100,vel_t=100,blendR=20)

    # 第三组关节位置和描述位姿
    JP14 = [9.586, -66.925, 85.589, -99.109, -103.403, -30.280]
    DP14 = [0]*6
    JP15 = [23.056, -59.187, 76.487, -102.155, -77.560, -30.250]
    DP15 = [0]*6
    JP16 = [28.028, -71.754, 91.463, -102.182, -102.361, -30.253]
    DP16 = [0]*6
    JP17 = [38.974, -62.622, 79.068, -102.543, -101.630, -30.253]
    DP17 = [0]*6
    JP18 = [-34.797, -72.641, 93.917, -104.961, -84.449, -30.287]
    DP18 = [0]*6
    JP19 = [-17.454, -58.309, 82.054, -111.034, -109.900, -30.241]
    DP19 = [0]*6
    JP20 = [-4.930, -72.469, 100.631, -109.906, -76.760, -10.947]
    DP20 = [0]*6
    JP21 = [3.021, -76.365, 81.332, -98.130, -68.530, -30.284]
    DP21 = [0]*6
    JP22 = [12.532, -94.241, 106.254, -87.131, -102.719, -30.227]
    DP22 = [0]*6

    # 正运动学计算
    error,DP14 = robot.GetForwardKin(JP14)
    error,DP15 = robot.GetForwardKin(JP15)
    error,DP16 = robot.GetForwardKin(JP16)
    error,DP17 = robot.GetForwardKin(JP17)
    error,DP18 = robot.GetForwardKin(JP18)
    error,DP19 = robot.GetForwardKin(JP19)
    error,DP20 = robot.GetForwardKin(JP20)
    error,DP21 = robot.GetForwardKin(JP21)
    error,DP22 = robot.GetForwardKin(JP22)

    # 运动指令
    robot.MoveJ(joint_pos=JP14, tool=0, user=0, vel=100, blenT=200)
    robot.Circle(desc_pos_p=JP15,tool_p=0,user_p=0,desc_pos_t=DP16,tool_t=0,user_t=0,vel_p=100,vel_t=100,oacc=100,blendR=20)
    robot.MoveJ(joint_pos=JP17, tool=0, user=0, vel=100, blenT=200)
    robot.MoveL(desc_pos=DP18, tool=0, user=0, vel=100, blendR=0)
    robot.MoveC(desc_pos_p=DP19, tool_p=0, user_p=0, desc_pos_t=DP20, tool_t=0, user_t=0, vel_p=100, vel_t=100,blendR=20)
    robot.MoveC(desc_pos_p=DP21, tool_p=0, user_p=0, desc_pos_t=DP22, tool_t=0, user_t=0, vel_p=100, vel_t=100,blendR=20)

    # 第四组关节位置和描述位姿
    JP23 = [9.586, -66.925, 85.589, -99.109, -103.403, -30.280]
    DP23 = [0]*6
    JP24 = [23.056, -59.187, 76.487, -102.155, -77.560, -30.250]
    DP24 = [0]*6
    JP25 = [28.028, -71.754, 91.463, -102.182, -102.361, -30.253]
    DP25 = [0]*6
    JP26 = [-11.207, -81.555, 110.050, -108.983, -74.292, -30.249]
    DP26 = [0]*6
    JP27 = [18.930, -70.987, 100.659, -115.974, -115.465, -30.231]
    DP27 = [0]*6
    JP28 = [32.493, -65.561, 86.053, -109.669, -103.427, -30.267]
    DP28 = [0]*6
    JP29 = [21.954, -87.113, 123.299, -109.730, -72.157, -9.013]
    DP29 = [0]*6
    JP30 = [19.084, -69.127, 104.304, -109.629, -106.997, -9.011]
    DP30 = [0]*6
    JP31 = [38.654, -60.146, 93.485, -109.637, -87.023, -8.989]
    DP31 = [0]*6

    # 正运动学计算
    error,DP23 = robot.GetForwardKin(JP23)
    error,DP24 = robot.GetForwardKin(JP24)
    error,DP25 = robot.GetForwardKin(JP25)
    error,DP26 = robot.GetForwardKin(JP26)
    error,DP27 = robot.GetForwardKin(JP27)
    error,DP28 = robot.GetForwardKin(JP28)
    error,DP29 = robot.GetForwardKin(JP29)
    error,DP30 = robot.GetForwardKin(JP30)
    error,DP31 = robot.GetForwardKin(JP31)

    # 运动指令
    robot.MoveL(desc_pos=DP23, tool=0, user=0, vel=100, blendR=20,blendMode=1)
    robot.Circle(desc_pos_p=DP24, tool_p=0, user_p=0, desc_pos_t=DP25, tool_t=0, user_t=0, vel_p=100, vel_t=100,
                 oacc=100, blendR=20)
    robot.Circle(desc_pos_p=DP26, tool_p=0, user_p=0, desc_pos_t=DP27, tool_t=0, user_t=0, vel_p=100, vel_t=100,
                 oacc=100, blendR=20)
    robot.MoveC(desc_pos_p=DP28, tool_p=0, user_p=0, desc_pos_t=DP29, tool_t=0, user_t=0, vel_p=100, vel_t=100,
                blendR=20)
    robot.Circle(desc_pos_p=DP30, tool_p=0, user_p=0, desc_pos_t=DP31, tool_t=0, user_t=0, vel_p=100, vel_t=100,
                 oacc=100, blendR=20)

    # 第五组关节位置和描述位姿
    JP32 = [38.654, -60.146, 93.485, -109.637, -87.023, -8.989]
    DP32 = [0]*6
    JP33 = [55.203, -69.138, 75.617, -103.969, -83.549, -0.001]
    DP33 = [0]*6
    JP34 = [57.646, -61.846, 59.286, -69.645, -99.735, 3.824]
    DP34 = [0]*6
    JP35 = [57.304, -61.380, 58.260, -67.641, -97.447, 2.685]
    DP35 = [0]*6
    JP36 = [57.297, -61.373, 58.250, -67.637, -97.448, 2.677]
    DP36 = [0]*6
    JP37 = [23.845, -108.202, 111.300, -80.971, -106.753, -30.246]
    DP37 = [0]*6
    JP38 = [23.845, -108.202, 111.300, -80.971, -106.753, -30.246]
    DP38 = [0]*6
    JP39 = [-10.503, -93.654, 111.333, -84.702, -103.479, -30.179]
    DP39 = [0]*6
    JP40 = [-30.623, -74.158, 89.844, -91.942, -97.060, -30.180]
    DP40 = [0]*6

    # 正运动学计算
    error,DP32 = robot.GetForwardKin(JP32)
    error,DP33 = robot.GetForwardKin(JP33)
    error,DP34 = robot.GetForwardKin(JP34)
    error,DP35 = robot.GetForwardKin(JP35)
    error,DP36 = robot.GetForwardKin(JP36)
    error,DP37 = robot.GetForwardKin(JP37)
    error,DP38 = robot.GetForwardKin(JP38)
    error,DP39 = robot.GetForwardKin(JP39)
    error,DP40 = robot.GetForwardKin(JP40)

    # 运动指令
    robot.MoveL(desc_pos=DP32, tool=0, user=0, vel=100, blendR=20, blendMode=1)
    robot.MoveJ(joint_pos=JP33, tool=0, user=0, vel=100, blenT=-1)
    robot.MoveL(desc_pos=DP34, tool=0, user=0, vel=100, blendR=20, blendMode=0)
    robot.MoveL(desc_pos=DP35, tool=0, user=0, vel=100, blendR=20, blendMode=0)
    robot.MoveL(desc_pos=DP36, tool=0, user=0, vel=100, blendR=20, blendMode=0)
    robot.MoveL(desc_pos=DP37, tool=0, user=0, vel=100, blendR=20, blendMode=0)
    robot.MoveL(desc_pos=DP38, tool=0, user=0, vel=100, blendR=20, blendMode=0)
    robot.MoveL(desc_pos=DP39, tool=0, user=0, vel=100, blendR=20, blendMode=0)
    robot.MoveJ(joint_pos=JP40, tool=0, user=0, vel=100, blenT=20)

    # 第六组关节位置和描述位姿
    JP50 = [-34.797, -72.641, 93.917, -104.961, -84.449, -30.287]
    DP50 = [0]*6
    JP41 = [-17.454, -58.309, 82.054, -111.034, -109.900, -30.241]
    DP41 = [0]*6
    JP42 = [-4.930, -72.469, 100.631, -109.906, -76.760, -10.947]
    DP42 = [0]*6
    JP43 = [9.586, -66.925, 85.589, -99.109, -103.403, -30.280]
    DP43 = [0]*6
    JP44 = [23.056, -59.187, 76.487, -102.155, -77.560, -30.250]
    DP44 = [0]*6
    JP45 = [28.028, -71.754, 91.463, -102.182, -102.361, -30.253]
    DP45 = [0]*6
    JP46 = [38.974, -62.622, 79.068, -102.543, -101.630, -30.253]
    DP46 = [0]*6

    # 正运动学计算
    error,DP50 = robot.GetForwardKin(JP50)
    error,DP41 = robot.GetForwardKin(JP41)
    error,DP42 = robot.GetForwardKin(JP42)
    error,DP43 = robot.GetForwardKin(JP43)
    error,DP44 = robot.GetForwardKin(JP44)
    error,DP45 = robot.GetForwardKin(JP45)
    error,DP46 = robot.GetForwardKin(JP46)

    # 运动指令
    robot.MoveL(desc_pos=DP50, tool=0, user=0, vel=100, blendR=20, blendMode=0)
    robot.MoveC(desc_pos_p=DP41, tool_p=0, user_p=0, desc_pos_t=DP42, tool_t=0, user_t=0, vel_p=100, vel_t=100,
                blendR=20)
    robot.MoveL(desc_pos=DP43, tool=0, user=0, vel=100, blendR=20, blendMode=0)
    robot.Circle(desc_pos_p=DP44, tool_p=0, user_p=0, desc_pos_t=DP45, tool_t=0, user_t=0, vel_p=100, vel_t=100,
                 oacc=100, blendR=20)
    robot.MoveL(desc_pos=DP46, tool=0, user=0, vel=100, blendR=20, blendMode=0)


# DragControl(robot)
# TestBlend(robot)
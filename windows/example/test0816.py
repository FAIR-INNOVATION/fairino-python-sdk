from time import sleep
from fairino import Robot
# A connection is established with the robot controller. A successful connection returns a robot object
# 与机器人控制器建立连接，连接成功返回一个机器人对象
robot = Robot.RPC('192.168.58.2')
import time


def TestMovePhy(self):
    # Define joint positions as lists [j1,j2,j3,j4,j5,j6]
    j1 = [-11.904, -99.669, 117.473, -108.616, -91.726, 74.256]
    j2 = [-45.615, -106.172, 124.296, -107.151, -91.282, 74.255]
    j3 = [-29.777, -84.536, 109.275, -114.075, -86.655, 74.257]
    j4 = [-31.154, -95.317, 94.276, -88.079, -89.740, 74.256]

    # Define Cartesian poses as lists [x,y,z,rx,ry,rz]
    desc_pos1 = [-419.524, -13.000, 351.569, -178.118, 0.314, 3.833]
    desc_pos2 = [-321.222, 185.189, 335.520, -179.030, -1.284, -29.869]
    desc_pos3 = [-487.434, 154.362, 308.576, 176.600, 0.268, -14.061]
    desc_pos4 = [-443.165, 147.881, 480.951, 179.511, -0.775, -15.409]
    desc_pos5 = [-385.268, -386.759, 238.349, 179.619, -2.046, 162.332]
    desc_pos6 = [-257.470, -566.986, 241.908, -177.038, -2.886, -176.577]
    desc_pos7 = [-190.925, -390.644, 240.374, 179.089, 0.019, 177.836]
    offset_pos = [0, 0, 0, 0, 0, 0]
    epos = [0, 0, 0, 0]  # External axis positions

    # Movement parameters
    tool = 0
    user = 0
    vel = 100.0
    acc = 200.0
    ovl = 100.0
    blendT = -1.0
    blendR = -1.0
    flag = 0
    search = 0

    robot.SetSpeed(20)

    # Execute movements
    rtn = robot.MoveL(desc_pos=desc_pos1, tool=tool, user=user, vel=vel, acc=acc, ovl=ovl, blendR=blendR,blendMode =  0,exaxis_pos= epos,search= search,offset_flag= flag,offset_pos= offset_pos,config= -1,velAccParamMode= 1)
    print(f"movel errcode: {rtn}")

    rtn = robot.MoveC(desc_pos_p=desc_pos3, tool_p=tool,user_p= user,vel_p= vel, acc_p= acc,exaxis_pos_p= epos,offset_flag_p= flag,offset_pos_p= offset_pos,
                      desc_pos_t=desc_pos4, tool_t=tool, user_t=user,vel_t= vel, acc_t=acc, exaxis_pos_t=epos,offset_flag_t= flag,offset_pos_t= offset_pos,ovl= ovl, blendR=blendR,config= -1,velAccParamMode= 1)
    print(f"movec errcode: {rtn}")

    rtn = robot.MoveL(desc_pos=desc_pos5, tool=tool, user=user, vel=vel, acc=acc, ovl=ovl, blendR=blendR,blendMode =  0,exaxis_pos= epos,search= search,offset_flag= flag,offset_pos= offset_pos,config= -1,velAccParamMode= 1)
    print(f"movel errcode: {rtn}")

    rtn = robot.Circle(desc_pos_p=desc_pos6, tool_p=tool,user_p= user,vel_p= vel, acc_p= acc,exaxis_pos_p= epos,
                       desc_pos_t=desc_pos7, tool_t=tool, user_t=user,vel_t= vel, acc_t=acc, exaxis_pos_t=epos
                       ,ovl= ovl, offset_flag= flag,offset_pos= offset_pos,oacc= 100,blendR= -1,config= -1,velAccParamMode= 1)
    print(f"circle errcode: {rtn}")

    robot.CloseRPC()
    return 0

TestMovePhy(robot)

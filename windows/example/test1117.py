from time import sleep

from fairino import Robot
from ctypes import sizeof
# A connection is established with the robot controller. A successful connection returns a robot object
# 与机器人控制器建立连接，连接成功返回一个机器人对象
robot = Robot.RPC('192.168.58.2')

def MoveL(self):
    # d1 = [-106.630,-337.120,332.017,-171.118,78.189,-83.801]
    # d2 = [31.938,-458.280,323.482,-119.544,60.039,-18.168]
    d1 = [-370.51,-311.729,462.956,-178.322,5.295,174.528]
    d2 = [-370.51,88.729,462.956,-178.322,5.295,174.528]

    offset_pos = [0,0,20,0,0,0]
    offset_flag = 1
    blendR = 1
    tool = 0
    user = 0
    vel = 60.0
    # robot.SetSpeed(20)

    rtn = robot.MoveL(desc_pos=d1,tool=tool,user=user,vel=vel,blendR=blendR,offset_pos=offset_pos,offset_flag=offset_flag)
    print(rtn)
    rtn = robot.MoveL(desc_pos=d2,tool=tool,user=user,vel=vel,blendR=blendR,offset_pos=offset_pos,offset_flag=offset_flag)
    print(rtn)

    robot.CloseRPC()
    return

MoveL(robot)
from time import sleep
from fairino import Robot
# A connection is established with the robot controller. A successful connection returns a robot object
# 与机器人控制器建立连接，连接成功返回一个机器人对象
robot = Robot.RPC('192.168.58.2')
import time


def TestImpedanceControl(self):
    j1 = [102.622, -135.990, 120.769, -73.950, -90.848, 35.507]
    j2 = [93.674, -80.062, 82.947, -92.199, -90.967, 26.559]
    desc_pos1 = [136.552, -149.799, 449.532, 179.817, -1.172, 157.123]
    desc_pos2 = [136.540, -561.048, 449.542, 179.819, -1.172, 157.122]
    offset_pos = [0.0] * 6
    epos = [0.0] * 4
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

    forceThreshold = [10.0, 10.0, 10.0, 1.0, 1.0, 1.0]
    m = [0.04,0.04,0.04,0.01,0.01,0.01]
    b = [0.1,0.1,0.1,0.08,0.08,0.08]
    k = [0.0] * 6

    rtn = robot.ImpedanceControlStartStop(1, 0, forceThreshold, m, b, k, 50, 50, 100, 100)
    print(f"ImpedanceControlStartStop errcode:{rtn}")
    rtn = robot.MoveJ(joint_pos=j1,tool= tool,user= user,vel= vel,acc= acc,ovl= ovl,exaxis_pos= epos,blendT= -1,offset_flag= 0,offset_pos= offset_pos)
    rtn = robot.MoveJ(joint_pos=j2,tool= tool,user= user,vel= vel,acc= acc,ovl= ovl,exaxis_pos= epos,blendT= -1,offset_flag= 0,offset_pos= offset_pos)
    rtn = robot.MoveJ(joint_pos=j1,tool= tool,user= user,vel= vel,acc= acc,ovl= ovl,exaxis_pos= epos,blendT= -1,offset_flag= 0,offset_pos= offset_pos)
    rtn = robot.MoveJ(joint_pos=j2,tool= tool,user= user,vel= vel,acc= acc,ovl= ovl,exaxis_pos= epos,blendT= -1,offset_flag= 0,offset_pos= offset_pos)
    print(f"movej errcode:{rtn}")

    robot.ImpedanceControlStartStop(0, 0, forceThreshold, m, b, k, 50, 50, 100, 100)

    robot.CloseRPC()
    return 0

TestImpedanceControl(robot)
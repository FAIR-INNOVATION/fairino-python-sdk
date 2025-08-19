from time import sleep

from fairino import Robot
# A connection is established with the robot controller. A successful connection returns a robot object
# 与机器人控制器建立连接，连接成功返回一个机器人对象
robot = Robot.RPC('192.168.58.2')
import time


def TestMoveJ(self):
    JP1 = [-44.185,-95.599,102.888,-100.999,-90.04,-54.095]
    JP2 = [39.128,-95.532,102.739,-101.114,-90.038,-54.095]
    error = robot.MoveJ(JP2, tool=0, user=0, vel=100)
    print("MoveJ return ", error)

def TestMoveL(self):
    DP1 = [-399.733,246.791,430.238,-177.855,-3.027,99.853]
    DP2 = [-292.353,-368.834,431.221,-177.737,-3.186,-176.836]
    error = robot.MoveL(DP1, tool=0, user=0, vel=100)
    print("MoveL return ", error)

def TestMoveC(self):
    DP1 = [-399.733, 246.791, 430.238, -177.855, -3.027, 99.853]
    DP2 = [-292.353, -368.834, 431.221, -177.737, -3.186, -176.836]

    DP = [-443.453,453.469,197.678,-179.228,-0.561,89.187]
    error = robot.MoveL(DP, tool=0, user=0, vel=100)
    print("MoveL return ", error)

    error = robot.MoveC(desc_pos_p=DP1,tool_p=0,user_p=0,desc_pos_t=DP2,tool_t=0,user_t=0)
    print("MoveC return ",error)

def Circle(self):
    DP3 = [-599.313,-314.327,365.756,179.445,0.718,163.133]
    DP4 = [-430.217,-83.687,370.316,177.476,3.436,141.619]
    error = robot.Circle(desc_pos_p=DP3,tool_p=0,user_p=0,desc_pos_t=DP4,tool_t=0,user_t=0,vel_p=100,vel_t=100)
    print("Circle return ",error)

def newspiral(self):
    """机器人NewSpiral测试"""
    offset_pos1 = [50, 0, 0, -30, 0, 0]
    offset_pos2 = [50, 0, 0, -5, 0, 0]
    epos = [0, 0, 0, 0]
    sp = [1, 5.0, 50.0, 10.0, 10.0, 0]
    tool = 0
    user = 0
    vel = 100.0
    acc = 100.0
    ovl = 100.0
    blendT = 0.0
    flag = 2
    joint_pos = [-11.904, -99.669, 117.473, -108.616, -91.726, 74.256]
    desc_pos = [-419.524, -13.000, 351.569, -178.118, 0.314, 3.833]
    rtn = robot.MoveJ(joint_pos=joint_pos, tool=tool, user=user, exaxis_pos=epos, blendT=blendT, offset_flag=flag,offset_pos=offset_pos1,vel=100)
    print(f"MoveJ return: {rtn}")
    sleep(2)
    rtn = robot.NewSpiral(desc_pos=desc_pos, tool=tool, user=user, param=sp, exaxis_pos=epos, offset_flag=flag,offset_pos=offset_pos2,vel=100)
    print(f"NewSpiral return: {rtn}")

def TestSpline(self):
    """样条运动测试函数"""
    joint_points = [
        [-11.904, -99.669, 117.473, -108.616, -91.726, 74.256],
        [-45.615, -106.172, 124.296, -107.151, -91.282, 74.255],
        [-61.954, -84.409, 108.153, -116.316, -91.283, 74.260],
        [-89.575, -80.276, 102.713, -116.302, -91.284, 74.267]
    ]
    offset_pos = [0] * 6
    epos = [0] * 4
    tool = user = 0
    vel = acc = ovl = 100.0
    blendT = -1.0
    flag = 0
    sleep(1)
    err1 = robot.MoveJ(joint_pos=joint_points[0],tool=tool, user=user,vel=vel)
    print(f"MoveJ return: {err1}")
    robot.SplineStart()
    robot.SplinePTP(joint_pos=joint_points[0],tool=tool, user=user,vel=100)
    robot.SplinePTP(joint_pos=joint_points[1],tool=tool, user=user,vel=100)
    robot.SplinePTP(joint_pos=joint_points[2],tool=tool, user=user,vel=100)
    robot.SplinePTP(joint_pos=joint_points[3],tool=tool, user=user,vel=100)
    robot.SplineEnd()

def TestNewSpline(self):
    j1 = [-11.904, -99.669, 117.473, -108.616, -91.726, 74.256]
    desc_pos1 = [-419.524, -13.000, 351.569, -178.118, 0.314, 3.833]
    desc_pos2 = [-321.222, 185.189, 335.520, -179.030, -1.284, -29.869]
    desc_pos3 = [-327.622, 402.230, 320.402, -178.067, 2.127, -46.207]
    desc_pos4 = [-104.066, 544.321, 327.023, -177.715, 3.371, -73.818]
    desc_pos5 = [-33.421, 732.572, 275.103, -177.907, 2.709, -79.482]
    offset_pos = [0, 0, 0, 0, 0, 0]
    epos = [0, 0, 0, 0]
    tool = 0
    user = 0
    vel = 100.0
    acc = 100.0
    ovl = 100.0
    blendT = -1.0
    flag = 0
    err1 = robot.MoveJ(joint_pos=j1, tool=tool, user=user, vel=vel)
    print(f"movej errcode:{err1}")
    robot.NewSplineStart(1, 2000)
    robot.NewSplinePoint(desc_pos=desc_pos1, tool=tool, user=user, vel=vel, lastFlag=-1, blendR=0)
    robot.NewSplinePoint(desc_pos=desc_pos2, tool=tool, user=user, vel=vel, lastFlag=-1, blendR=0)
    robot.NewSplinePoint(desc_pos=desc_pos3, tool=tool, user=user, vel=vel, lastFlag=-1, blendR=0)
    robot.NewSplinePoint(desc_pos=desc_pos4, tool=tool, user=user, vel=vel, lastFlag=-1, blendR=0)
    robot.NewSplinePoint(desc_pos=desc_pos5, tool=tool, user=user, vel=vel, lastFlag=-1, blendR=0)
    robot.NewSplineEnd()

def ExtAxisSyncMove(self):
        joint_safe = [116.142, -83.505, 53.915, -64.393, -87.817, -49.868]
        joint_pos1 = [107.108, -93.262, 90.994, -86.002, -89.095, -50.897]
        joint_pos2 = [101.428, -60.890, 57.914, -63.062, -85.489, -50.897]
        joint_pos3 = [124.486, -97.485, 95.883, -90.374, -91.062, -49.860]
        joint_pos4 = [113.734, -76.571, 78.254, -93.291, -91.100, -49.867]
        joint_pos5 = [116.544, -63.563, 63.923, -88.894, -83.999, -49.869]

        desc_safe = [334.834, -416.117, 377.662, -175.764, -1.640, -103.974]
        desc_pos1 = [238.956, -408.157, 189.101, 179.611, 1.912, -112.015]
        desc_pos2 = [236.316, -495.487, 138.006, 167.595, 21.130, -120.956]
        desc_pos3 = [335.706, -321.654, 188.246, -179.537, -2.195, -95.681]
        desc_pos4 = [330.540, -517.439, 156.875, -179.804, -1.939, -106.418]
        desc_pos5 = [430.747, -540.471, 129.780, -176.351, 4.987, -103.505]

        epos_safe = [0.000, 0.000, 0.000, 0.000]
        epos1 = [0.000, 0.000, 0.000, 0.000]
        epos2 = [-24.972, 0.000, 0.000, 0.000]
        epos3 = [14.982, 0.000, 0.000, 0.000]
        epos4 = [-10.026, 0.000, 0.000, 0.000]
        epos5 = [-20.015, 0.000, 0.000, 0.000]

        offset_pos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        tool = 1
        user = 0
        vel = 100
        acc = 100
        ovl = 100
        blendT = -1

        robot.MoveJ(joint_pos=joint_safe,tool=tool,user=user,vel=vel, acc=acc, ovl=ovl, exaxis_pos=epos_safe, blendT=blendT, offset_flag=0, offset_pos=offset_pos)
        robot.ExtAxisMove(pos=epos_safe, ovl=100, blend=-1)

        rtn = robot.ExtAxisSyncMoveJ(joint_pos=joint_pos1,tool= tool,user= user,vel= vel,acc= acc,ovl= ovl,exaxis_pos= epos1,blendT= blendT,offset_flag= 0,offset_pos= offset_pos)
        rtn = robot.ExtAxisSyncMoveJ(joint_pos=joint_pos2,tool= tool,user= user,vel= vel,acc= acc,ovl= ovl,exaxis_pos= epos2,blendT= blendT,offset_flag= 0,offset_pos= offset_pos)
        print(f"ExtAxisSyncMoveJ rtn is {rtn}")

        robot.MoveJ(joint_pos=joint_safe, tool=tool,user=user,vel=vel, acc=acc, ovl=ovl, exaxis_pos=epos_safe, blendT=blendT, offset_flag=0, offset_pos=offset_pos)
        robot.ExtAxisMove(pos=epos_safe, ovl=100, blend=-1)
        rtn = robot.ExtAxisSyncMoveJ(joint_pos=joint_pos1,tool= tool,user= user,vel= vel,acc= acc,ovl= ovl,exaxis_pos= epos1,blendT= blendT,offset_flag= 0,offset_pos= offset_pos)
        rtn = robot.ExtAxisSyncMoveL(desc_pos=desc_pos3,tool= tool,user= user,vel= vel,acc= acc,ovl= ovl,blendR= -1,exaxis_pos= epos3, offset_flag=0, offset_pos=offset_pos)
        print(f"ExtAxisSyncMoveL rtn is {rtn}")

        robot.MoveJ(joint_pos=joint_safe, tool=tool,user=user,vel=vel, acc=acc, ovl=ovl, exaxis_pos=epos_safe, blendT=blendT, offset_flag=0, offset_pos=offset_pos)
        robot.ExtAxisMove(pos=epos_safe, ovl=100, blend=-1)
        rtn = robot.ExtAxisSyncMoveJ(joint_pos=joint_pos1,tool= tool,user= user,vel= vel,acc= acc,ovl= ovl,exaxis_pos= epos1,blendT= blendT,offset_flag= 0,offset_pos= offset_pos)
        rtn = robot.ExtAxisSyncMoveC(desc_pos_p=desc_pos4,tool_p= tool,user_p= user,vel_p= vel,acc_p= acc,exaxis_pos_p= epos4,offset_flag_p= 0,offset_pos_p= offset_pos,
                                     desc_pos_t=desc_pos5,tool_t= tool,user_t= user,vel_t= vel,acc_t= acc,exaxis_pos_t= epos5,offset_flag_t= 0,offset_pos_t= offset_pos,ovl= ovl,blendR= -1,config=-1)
        print(f"ExtAxisSyncMoveC rtn is {rtn}")

        robot.CloseRPC()

# i=1
# while i<101:
#     print("第",i,"轮运行")
#     TestMoveJ(robot)
#     TestMoveL(robot)
#     TestMoveC(robot)
#     Circle(robot)
#     newspiral(robot)
#     TestSpline(robot)
#     TestNewSpline(robot)
#     i = i + 1


ExtAxisSyncMove(robot)
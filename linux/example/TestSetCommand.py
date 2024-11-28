from fairino import Robot
import time
# A connection is established with the robot controller. A successful connection returns a robot object
robot = Robot.RPC('192.168.58.2')
JP1 = [117.408,-86.777,81.499,-87.788,-92.964,92.959]
DP1 = [327.359,-420.973,518.377,-177.199,3.209,114.449]

JP2 = [72.515,-86.774,81.525,-87.724,-91.964,92.958]
DP2 = [-65.169,-529.17,518.018,-177.189,3.119,69.556]

# JP2_h = [72.515,-86.774,81.525,-87.724,-91.964,92.958]
DP2_h = [-65.169,-529.17,528.018,-177.189,3.119,69.556]

JP3 = [89.281,-102.959,81.527,-69.955,-86.755,92.958]
DP3 = [102.939,-378.069,613.165,176.687,1.217,86.329]

desc = [0,0,0,0,0,0]

def setspeed(self):
    """机器人SetSpeed测试"""
    error = robot.SetSpeed(vel=40)
    print("SetSpeed return ",error)

def setoaccscale(self):
    """机器人SetOaccScale测试"""
    error = robot.SetOaccScale(acc=20)
    robot.MoveJ(JP1, tool=0, user=0, vel=80)
    error = robot.SetOaccScale(acc=100)
    robot.MoveJ(JP2, tool=0, user=0, vel=80)
    print("SetOaccScale return ",error)

def setsysvarvalue(self):
    """机器人SetSysVarValue测试"""
    error = robot.SetSysVarValue(id=1,value=10)
    error,num = robot.GetSysVarValue(id=1)
    print("SetSysVarValue return ", error)
    print("id=1的系统变量值为", num)

def settoolpoint(self):
    """机器人六点法计算工具坐标系"""
    j1 = [-89.407, -148.279, -83.169, -45.689, 133.689, 41.705]
    j2 = [-67.595, -143.7, -88.006, -48.514, 57.073, 56.189]
    j3 = [-88.229, -152.355, -67.815, -78.07, 129.029, 58.739]
    j4 = [-77.528, -141.519, -89.826, -37.184, 90.274, 41.769]
    j5 = [-76.744, -138.219, -97.714, -32.595, 90.255, 42.558]
    j6 = [-77.595, -138.454, -90.065, -40.014, 90.275, 41.709]

    error = robot.MoveJ(j1, tool=0, user=0, vel=30)
    robot.SetToolPoint(point_num=1)
    error = robot.MoveJ(j2, tool=0, user=0, vel=30)
    robot.SetToolPoint(point_num=2)
    error = robot.MoveJ(j3, tool=0, user=0, vel=30)
    robot.SetToolPoint(point_num=3)
    error = robot.MoveJ(j4, tool=0, user=0, vel=30)
    robot.SetToolPoint(point_num=4)
    error = robot.MoveJ(j5, tool=0, user=0, vel=30)
    robot.SetToolPoint(point_num=5)
    error = robot.MoveJ(j6, tool=0, user=0, vel=30)
    robot.SetToolPoint(point_num=6)

    error,toolpoint = robot.ComputeTool()
    print("六点法设置工具坐标系 return ", error)
    print("六点法设置工具坐标系为：", toolpoint)

def settcp4refpoint(self):
    """机器人四点法计算工具坐标系"""
    j1 = [-89.407, -148.279, -83.169, -45.689, 133.689, 41.705]
    j2 = [-67.595, -143.7, -88.006, -48.514, 57.073, 56.189]
    j3 = [-88.229, -152.355, -67.815, -78.07, 129.029, 58.739]
    j4 = [-77.528, -141.519, -89.826, -37.184, 90.274, 41.769]

    error = robot.MoveJ(j1, tool=0, user=0, vel=20)
    robot.SetTcp4RefPoint(point_num=1)
    error = robot.MoveJ(j2, tool=0, user=0, vel=20)
    robot.SetTcp4RefPoint(point_num=2)
    error = robot.MoveJ(j3, tool=0, user=0, vel=20)
    robot.SetTcp4RefPoint(point_num=3)
    error = robot.MoveJ(j4, tool=0, user=0, vel=20)
    robot.SetTcp4RefPoint(point_num=4)

    error,toolpoint = robot.ComputeTcp4()
    print("四点法设置工具坐标系 return ", error)
    print("四点法设置工具坐标系为：", toolpoint)

def settoolcoord(self):
    """机器人设置工具坐标系"""
    coord = [0, 0, 200, 0, 0, 0]
    error = robot.SetToolCoord(id=4,t_coord=coord,type=0,install=0)
    print("SetToolCoord return ", error)
    error = robot.SetToolList(id=4,t_coord=coord,type=0,install=0)
    print("SetToolList return ", error)

def setextcppoint(self):
    """机器人设置外部工具坐标系"""
    j1 = [-84.787,-152.056,-75.689,-37.899,94.486,41.709]
    j2 = [-79.438,-152.139,-75.634,-37.469,94.065,47.058]
    j3 = [-84.788,-145.179,-77.119,-43.345,94.487,41.709]

    error = robot.MoveJ(j1, tool=0, user=0, vel=20)
    robot.SetExTCPPoint(point_num=1)
    error = robot.MoveJ(j2, tool=0, user=0, vel=20)
    robot.SetExTCPPoint(point_num=2)
    error = robot.MoveJ(j3, tool=0, user=0, vel=20)
    robot.SetExTCPPoint(point_num=3)

    error, extoolpoint = robot.ComputeExTCF()
    print("三点法设置外部工具坐标系 return ", error)
    print("三点法设置外部工具坐标系为：", extoolpoint)

def setextoolcoord(self):
    """机器人设置外部工具坐标系"""
    coord = [0, 0, 200, 0, 0, 0]
    error = robot.SetExToolCoord(id=4,etcp=coord,etool=coord)
    print("SetExToolCoord return ", error)
    error = robot.SetExToolList(id=4,etcp=coord,etool=coord)
    print("SetExToolList return ", error)


def setwobjcoordpoint(self):
    """机器人三点法设置工件坐标系"""
    j1 = [-84.787, -152.056, -75.689, -37.899, 94.486, 41.709]
    j2 = [-79.438, -152.139, -75.634, -37.469, 94.065, 47.058]
    j3 = [-84.788, -145.179, -77.119, -43.345, 94.487, 41.709]

    robot.MoveJ(j1, tool=0, user=0, vel=20)
    robot.SetWObjCoordPoint(point_num=1)
    robot.MoveJ(j2, tool=0, user=0, vel=20)
    robot.SetWObjCoordPoint(point_num=2)
    robot.MoveJ(j3, tool=0, user=0, vel=20)
    robot.SetWObjCoordPoint(point_num=3)

    error, wobjcoord = robot.ComputeWObjCoord(method=0)
    print("三点法设置工件坐标系 return ", error)
    print("三点法设置工件坐标系为：", wobjcoord)

    error = robot.SetWObjCoord(id=4, w_coord=wobjcoord)
    print("SetWObjCoord return ", error)
    error = robot.SetWObjList(id=4,w_coord=wobjcoord)
    print("SetWObjList return ", error)

def setloadweight(self):
    """机器人设置负载及负载中心"""
    robot.SetLoadWeight(weight=1)
    error = robot.SetLoadCoord(x=1,y=2,z=3)
    print("SetLoadCoord return ", error)

def setrobotinstallpos(self):
    """机器人安装方式-固定安装"""
    error = robot.SetRobotInstallPos(method=0)
    print("SetRobotInstallPos return ", error)

def setrobotinstallangle(self):
    """机器人安装角度"""
    error = robot.SetRobotInstallAngle(yangle=30,zangle=30)
    print("SetRobotInstallAngle return ", error)



# setspeed(robot)
# setoaccscale(robot)
# setsysvarvalue(robot)

##################还没测#######################
# settoolpoint(robot)
# settcp4refpoint(robot)
# settoolcoord(robot)
# setextcppoint(robot)
# setextoolcoord(robot)
setwobjcoordpoint(robot)
############################################


# setloadweight(robot)
# setrobotinstallpos(robot)
# setrobotinstallangle(robot)
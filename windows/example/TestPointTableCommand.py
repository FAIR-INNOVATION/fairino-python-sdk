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


def pointtabledownload(self):
    """点位表下载"""
    error = robot.PointTableDownLoad(point_table_name="point_table_test1.db",save_file_path="D://zUP/")
    print("PointTableDownLoad return ",error)

def pointtableupload(self):
    """点位表上传"""
    error = robot.PointTableUpLoad(point_table_file_path="D://zUP/point_table_test2.db")
    print("PointTableUpLoad return ",error)

def pointtableswitch(self):
    """点位表切换"""
    error = robot.PointTableSwitch(point_table_name="point_table_test1.db")
    print("PointTableSwitch return ",error)

def pointtableupdatelua(self):
    """点位表更新"""
    error = robot.PointTableUpdateLua(point_table_name="point_table_test1.db",lua_file_name="1010Test.lua")
    print("PointTableUpdateLua return ", error)


# pointtabledownload(robot)
# pointtableupload(robot)
# pointtableswitch(robot)
pointtableupdatelua(robot)
from fairino import Robot
import time
# A connection is established with the robot controller. A successful connection returns a robot object
import time
import threading

# 与机器人控制器建立连接，连接成功返回一个机器人对象
robot = Robot.RPC('192.168.58.2')

def test(self):
    for i in range(10):
        print("DataPackageDownload start")
        rtn = robot.DataPackageDownload("D://zDOWN/")
        print(f"DataPackageDownload rtn is {rtn}  times {i}")

    for i in range(10):
        print("AllDataSourceDownload start")
        rtn = robot.AllDataSourceDownload("D://zDOWN/")
        print(f"AllDataSourceDownload rtn is {rtn}  times {i}")

    for i in range(10):
        print("RbLogDownload start")
        rtn = robot.RbLogDownload("D://zDOWN/")
        print(f"RbLogDownload rtn is {rtn}  times {i}")

def testdown(self):
    print("RbLogDownload start")
    rtn = robot.RbLogDownload("D://zDOWN/")
    print(f"RbLogDownload rtn is {rtn}")

    # print("AllDataSourceDownload start")
    # rtn = robot.AllDataSourceDownload("D://zDOWN/")
    # print(f"AllDataSourceDownload rtn is {rtn}")

    # print("DataPackageDownload start")
    # rtn = robot.DataPackageDownload("D://zDOWN/")
    # print(f"DataPackageDownload rtn is {rtn}")



    # rtn = robot.ShutDownRobotOS()
    # print(f"ShutDownRobotOS rtn is {rtn}")

    # error,SN = robot.GetRobotSN()
    # print(f"robot SN is {SN}")

# test(robot)
testdown(robot)
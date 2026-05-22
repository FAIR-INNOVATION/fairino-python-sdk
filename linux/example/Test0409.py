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

    print("=" * 60)
    print("开始测试日志下载功能")
    print(f"下载目录: D://zDOWN/")
    print("=" * 60)

    for i in range(100):
        print(f"\n第 {i + 1} 次下载开始...")

        # 记录开始时间
        start = time.time()

        # 执行下载
        rtn = robot.RbLogDownload("D://zDOWN/")

        # 计算耗时
        elapsed = time.time() - start

        # 打印结果
        if rtn == 0:
            print(f"RbLog下载成功，耗时: {elapsed:.3f} 秒")
        else:
            print(f"RbLog下载失败, 错误码: {rtn}")

    print("\n" + "=" * 60)
    print("测试完成")
    print("=" * 60)
    time.sleep(100)
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
from time import sleep
import time
from fairino import Robot
# 与机器人控制器建立连接
robot = Robot.RPC('192.168.58.2')

def test_robot_stop_on_com_disc(self):
    # 初始化参数
    enable = False
    confirm_time = 0

    # 设置通信断开时机器人停止功能
    rtn = robot.SetRobotStopOnComDisc(0, True, 330)
    print(f"SetRobotStopOnComDisc index0: {rtn}")

    rtn = robot.SetRobotStopOnComDisc(1, True, 550)
    print(f"SetRobotStopOnComDisc index1: {rtn}")

    rtn = robot.SetRobotStopOnComDisc(2, True, 110)
    print(f"SetRobotStopOnComDisc index2: {rtn}")

    rtn = robot.SetRobotStopOnComDisc(3, True, 220)
    print(f"SetRobotStopOnComDisc index3: {rtn}")

    # 获取通信断开时机器人停止设置
    rtn, enable, confirm_time = robot.GetRobotStopOnComDisc(0)
    print(f"GetRobotStopOnComDisc 8080 rtn {rtn}; enable is {enable}; confirm time is {confirm_time}")

    rtn, enable, confirm_time = robot.GetRobotStopOnComDisc(1)
    print(f"GetRobotStopOnComDisc 80803 rtn {rtn}; enable is {enable}; confirm time is {confirm_time}")

    rtn, enable, confirm_time = robot.GetRobotStopOnComDisc(2)
    print(f"GetRobotStopOnComDisc 20002 rtn {rtn}; enable is {enable}; confirm time is {confirm_time}")

    rtn, enable, confirm_time = robot.GetRobotStopOnComDisc(3)
    print(f"GetRobotStopOnComDisc 20004 rtn {rtn}; enable is {enable}; confirm time is {confirm_time}")

    # 关闭RPC连接
    robot.CloseRPC()
    return 0

test_robot_stop_on_com_disc(robot)
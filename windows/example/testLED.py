from time import sleep
import time
from fairino import Robot

# 与机器人控制器建立连接
robot = Robot.RPC('192.168.58.2')


def testled(self):
    # 设置用户LED灯颜色
    # 参数顺序: R, G, B (红, 绿, 蓝)

    # 白色 (红绿蓝全亮)
    robot.SetUserLEDColor(True, True, True)
    time.sleep(1)

    # 关闭所有灯
    robot.SetUserLEDColor(False, False, False)
    time.sleep(1)

    # 红色 (仅红灯亮)
    robot.SetUserLEDColor(True, False, False)
    time.sleep(1)

    # 绿色 (仅绿灯亮)
    robot.SetUserLEDColor(False, True, False)
    time.sleep(1)

    # 蓝色 (仅蓝灯亮)
    robot.SetUserLEDColor(False, False, True)

    # 关闭连接
    robot.CloseRPC()

testled(robot)
from time import sleep
import time
from fairino import Robot

# 与机器人控制器建立连接
robot = Robot.RPC('192.168.58.2')

def TestOriginPointWeave(self):
    time.sleep(2)
    # 初始化关节位置、外部轴和偏移
    j = [39.886, -98.580, -124.032, -47.393, 90.000, 40.842]
    epos = [0, 0, 0, 0]
    offset_pos = [0, 0, 0, 0, 0, 0]

    # 参考点位置 [x, y, z, rx, ry, rz]
    refPoint = [400.021, 300.022, 299.996, 179.997, -0.003, -90.956]

    # 移动到起始位置
    robot.MoveJ(joint_pos=j, tool=1, user=0, vel=100, acc=100, ovl=100,
                exaxis_pos=epos, blendT=-1, offset_flag=0, offset_pos=offset_pos)

    # 第一次摆动：绝对坐标系（tool=0），模式0
    robot.OriginPointWeaveStart(0, 0, refPoint, 3)
    robot.MoveStationary()
    robot.OriginPointWeaveEnd()

    time.sleep(2)

    # 再次移动到起始位置
    robot.MoveJ(joint_pos=j, tool=1, user=0, vel=100, acc=100, ovl=100,
                exaxis_pos=epos, blendT=-1, offset_flag=0, offset_pos=offset_pos)

    # 第二次摆动：绝对坐标系（tool=0），模式1
    robot.OriginPointWeaveStart(0, 1, refPoint, 3)
    robot.MoveStationary()
    robot.OriginPointWeaveEnd()

    # 关闭连接
    robot.CloseRPC()
    time.sleep(1)

TestOriginPointWeave(robot)
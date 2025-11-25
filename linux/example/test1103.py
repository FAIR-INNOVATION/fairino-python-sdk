from time import sleep
from fairino import Robot
from ctypes import sizeof
# A connection is established with the robot controller. A successful connection returns a robot object
# 与机器人控制器建立连接，连接成功返回一个机器人对象
robot = Robot.RPC('192.168.58.2')

def TestIntersectLineMove(self):
    mainPoint = [[0.0] * 6 for _ in range(6)]
    piecePoint = [[0.0] * 6 for _ in range(6)]

    mainPoint[0] = [144.084, 512.064, 8.899, -58.958, 40.838, 23.295]
    mainPoint[1] = [132.150, 512.638, 28.157, -58.626, 40.788, 24.966]
    mainPoint[2] = [150.155, 514.479, 74.107, -47.740, 40.410, 27.552]
    mainPoint[3] = [188.346, 518.501, 73.946, -17.227, 60.139, 15.265]
    mainPoint[4] = [206.811, 520.214, 52.966, -18.198, 60.381, 12.624]
    mainPoint[5] = [203.002, 518.627, 19.028, -23.830, 61.410, 8.343]

    piecePoint[0] = [190.428, 480.862, 102.236, 8.966, 46.472, 35.582]
    piecePoint[1] = [201.770, 510.904, 101.912, 12.079, 66.897, 26.452]
    piecePoint[2] = [186.344, 533.294, 102.866, 1.980, 62.882, 25.094]
    piecePoint[3] = [162.969, 537.537, 103.015, -22.013, 46.227, 28.606]
    piecePoint[4] = [139.465, 510.090, 103.505, -23.996, 33.774, 41.829]
    piecePoint[5] = [168.329, 475.251, 102.325, 4.241, 42.293, 38.624]

    tool = 4
    wobj = 0
    vel = 100.0
    acc = 100.0
    ovl = 10.0
    oacc = 10.0
    moveType = 1
    moveDirection = 1

    rtn = robot.MoveToIntersectLineStart(mainPoint, piecePoint, tool, wobj, vel, acc, ovl, oacc, moveType)
    print(f"MoveToIntersectLineStart rtn is {rtn}")
    rtn = robot.MoveIntersectLine(mainPoint, piecePoint, tool, wobj, vel, acc, ovl, oacc, moveDirection)
    print(f"MoveIntersectLine rtn is {rtn}")

    robot.CloseRPC()
    return

TestIntersectLineMove(robot)
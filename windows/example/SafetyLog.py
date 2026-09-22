from time import sleep
import time
from fairino import Robot

# 与机器人控制器建立连接
robot = Robot.RPC('192.168.58.2')


def test(self):

    # while 1:
    error = robot.SetAnticollision(mode=0, level=[2.0, 2.0, 2.0, 2.0, 2.0, 2.0], config=1)
    print(f"SetAnticollision: error={error}")

    error = robot.SetCollisionStrategy(strategy=0, safeTime=1000, safeDistance=150,
                                       safetyMargin=[10, 10, 10, 10, 10, 10])
    print(f"SetCollisionStrategy: error={error}")


test(robot)
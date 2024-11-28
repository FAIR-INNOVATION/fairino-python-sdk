from fairino import Robot
import random
import time

# 与机器人控制器建立连接，连接成功返回一个机器人对象
robot = Robot.RPC('192.168.58.2')

n=1
while n<10:
    print("year",robot.robot_state_pkg.year)
    print("mouth",robot.robot_state_pkg.mouth)
    print("day",robot.robot_state_pkg.day)
    print("hour",robot.robot_state_pkg.hour)
    print("minute",robot.robot_state_pkg.minute)
    print("second",robot.robot_state_pkg.second)
    print("millisecond",robot.robot_state_pkg.millisecond)
    n=n+1
    time.sleep(0.5)
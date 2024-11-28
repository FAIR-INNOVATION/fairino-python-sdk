from fairino import Robot
import random
import time

# 与机器人控制器建立连接，连接成功返回一个机器人对象
robot = Robot.RPC('192.168.58.2')

error = robot.ResetAllError()
print("ResetAllError",error)

time.sleep(1)
n=0
while n<5:
    time.sleep(0.5)
    for i in range(1):

        print("auxState.servoState:", robot.robot_state_pkg.auxState.servoState )
        print("auxState.servoState_emergency stop:", (robot.robot_state_pkg.auxState.servoState >> 7)& 0x01)
    n=n+1
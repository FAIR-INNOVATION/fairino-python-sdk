from fairino import Robot
import random
import time

# 与机器人控制器建立连接，连接成功返回一个机器人对象
robot = Robot.RPC('192.168.58.2')
robot.LoggerInit(output_model=0)
robot.SetLoggerLevel(4)

# error = robot.SoftwareUpgrade("D://Desktop/software.tar.gz",True)
# print("SoftwareUpgrade错误码:",error)

error = robot.SoftwareUpgrade("D://Desktop/software.tar.gz",False)
print("SoftwareUpgrade错误码:",error)

while robot.GetSoftwareUpgradeState()!=100:
    print("GetSoftwareUpgradeState:",robot.GetSoftwareUpgradeState())
    time.sleep(0.5)
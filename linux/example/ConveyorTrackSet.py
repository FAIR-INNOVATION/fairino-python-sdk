from fairino import Robot

"""
    传送带参数设置指令
"""


import time
# 与机器人控制器建立连接，连接成功返回一个机器人对象
robot = Robot.RPC('192.168.58.2')
#参数配置
param=[1,10000,200,0,0,20]
ret = robot.ConveyorSetParam(param)
print("传送带参数配置错误码",ret)
time.sleep(1)
#抓取点补偿
comp = [0.00, 0.00, 0.00]
ret1 = robot.ConveyorCatchPointComp(comp)
print("传动带抓取点补偿错误码",ret1)

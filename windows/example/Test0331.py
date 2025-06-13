from fairino import Robot
import time
# A connection is established with the robot controller. A successful connection returns a robot object
import time
import threading

# 与机器人控制器建立连接，连接成功返回一个机器人对象
robot = Robot.RPC('192.168.58.2')

def AccSmooth_MoveJ(self):
    """加速度平滑-MoveJ"""
    JP1 = [88.927,-85.834,80.289,-85.561,-91.388,108.718]
    DP1 = [88.739,-527.617,514.939,-179.039,1.494,70.209]

    JP2 = [27.036,-83.909,80.284,-85.579,-90.027,108.604]
    DP2 = [-433.125,-334.428,497.139,-179.723,-0.745,8.437]
    error = robot.AccSmoothStart(saveFlag=0)
    print("AccSmoothStart return:",error)
    error = robot.MoveJ(JP1, tool=0, user=0, vel=100)
    error = robot.MoveJ(JP2, tool=0, user=0, vel=100)
    error = robot.MoveJ(JP1, tool=0, user=0, vel=100)
    error = robot.MoveJ(JP2, tool=0, user=0, vel=100)
    error = robot.AccSmoothEnd(saveFlag=0)
    print("AccSmoothEnd return:", error)

def AccSmooth_MoveL(self):
    """加速度平滑-MoveL"""
    JP1 = [88.927,-85.834,80.289,-85.561,-91.388,108.718]
    DP1 = [88.739,-527.617,514.939,-179.039,1.494,70.209]

    JP2 = [27.036,-83.909,80.284,-85.579,-90.027,108.604]
    DP2 = [-433.125,-334.428,497.139,-179.723,-0.745,8.437]
    error = robot.AccSmoothStart(saveFlag=0)
    print("AccSmoothStart return:",error)
    error = robot.MoveL(DP1, tool=0, user=0, vel=100)
    error = robot.MoveL(DP2, tool=0, user=0, vel=100)
    error = robot.MoveL(DP1, tool=0, user=0, vel=100)
    error = robot.MoveL(DP2, tool=0, user=0, vel=100)
    error = robot.AccSmoothEnd(saveFlag=0)
    print("AccSmoothEnd return:", error)

def AccSmooth_MoveC(self):
    """加速度平滑-MoveC"""
    JP1 = [88.927,-85.834,80.289,-85.561,-91.388,108.718]
    DP1 = [88.739,-527.617,514.939,-179.039,1.494,70.209]

    JP2 = [27.036,-83.909,80.284,-85.579,-90.027,108.604]
    DP2 = [-433.125,-334.428,497.139,-179.723,-0.745,8.437]

    JP3 = [60.219,-94.324,62.906,-62.005,-87.159,108.598]
    DP3 = [-112.215,-409.323,686.497,176.217,2.338,41.625]
    error = robot.AccSmoothStart(saveFlag=0)
    print("AccSmoothStart return:",error)
    error = robot.MoveC(desc_pos_p=DP3, tool_p=0, user_p=0, desc_pos_t=DP1, tool_t=0, user_t=0, vel_p=100, vel_t=100)
    # error = robot.MoveC(desc_pos_p=DP3, tool_p=0, user_p=0, desc_pos_t=DP2, tool_t=0, user_t=0, vel_p=100, vel_t=100)
    error = robot.AccSmoothEnd(saveFlag=0)
    print("AccSmoothEnd return:", error)

def AccSmooth_Circle(self):
    """加速度平滑-Circle"""
    JP1 = [88.927,-85.834,80.289,-85.561,-91.388,108.718]
    DP1 = [88.739,-527.617,514.939,-179.039,1.494,70.209]

    JP2 = [27.036,-83.909,80.284,-85.579,-90.027,108.604]
    DP2 = [-433.125,-334.428,497.139,-179.723,-0.745,8.437]

    JP3 = [60.219,-94.324,62.906,-62.005,-87.159,108.598]
    DP3 = [-112.215,-409.323,686.497,176.217,2.338,41.625]
    error = robot.AccSmoothStart(saveFlag=0)
    print("AccSmoothStart return:",error)
    error = robot.Circle(desc_pos_p=DP3, tool_p=0, user_p=0, desc_pos_t=DP2, tool_t=0, user_t=0, vel_p=100, vel_t=100)
    error = robot.AccSmoothEnd(saveFlag=0)
    print("AccSmoothEnd return:", error)

# AccSmooth_MoveJ(robot)
# AccSmooth_MoveL(robot)
# AccSmooth_MoveC(robot)
AccSmooth_Circle(robot)

from time import sleep
import time
from fairino import Robot

# 与机器人控制器建立连接
robot = Robot.RPC('192.168.58.2')


def TestIOConfig(self):
    # 设置和获取DI配置
    setDIConfig = [1, 2, 3, 4, 5, 6, 7, 8]
    getDIConfig = [0] * 8
    rtn = robot.SetDIConfig(setDIConfig)
    print(f"SetDIConfig rtn is {rtn}")
    rtn, getDIConfig = robot.GetDIConfig()
    print(f"GetDIConfig rtn is {rtn}, value is {getDIConfig[0]} {getDIConfig[1]} {getDIConfig[2]} {getDIConfig[3]} {getDIConfig[4]} {getDIConfig[5]} {getDIConfig[6]} {getDIConfig[7]}")

    # 设置和获取DO配置
    setDOConfig = [9, 10, 11, 12, 13, 14, 15, 16]
    getDOConfig = [0] * 8
    rtn = robot.SetDOConfig(setDOConfig)
    print(f"SetDOConfig rtn is {rtn}")
    rtn, getDOConfig = robot.GetDOConfig()
    print(f"GetDOConfig rtn is {rtn}, value is {getDOConfig[0]} {getDOConfig[1]} {getDOConfig[2]} {getDOConfig[3]} {getDOConfig[4]} {getDOConfig[5]} {getDOConfig[6]} {getDOConfig[7]}")

    # 设置和获取工具DI配置
    setToolDIConfig = [17, 18]
    getToolDIConfig = [0] * 2
    rtn = robot.SetToolDIConfig(setToolDIConfig)
    print(f"SetToolDIConfig rtn is {rtn}")
    rtn, getToolDIConfig = robot.GetToolDIConfig()
    print(f"GetToolDIConfig rtn is {rtn}, value is {getToolDIConfig[0]} {getToolDIConfig[1]}")

    # 设置和获取DI电平配置（0: 低电平有效, 1: 高电平有效）
    setDIConfigLevel = [1, 1, 1, 1, 0, 0, 0, 0]
    getDIConfigLevel = [0] * 8
    rtn = robot.SetDIConfigLevel(setDIConfigLevel)
    print(f"SetDIConfigLevel rtn is {rtn}")
    rtn, getDIConfigLevel = robot.GetDIConfigLevel()
    print(f"GetDIConfigLevel rtn is {rtn}, value is {getDIConfigLevel[0]} {getDIConfigLevel[1]} {getDIConfigLevel[2]} {getDIConfigLevel[3]} {getDIConfigLevel[4]} {getDIConfigLevel[5]} {getDIConfigLevel[6]} {getDIConfigLevel[7]}")

    # 设置和获取DO电平配置（0: 低电平有效, 1: 高电平有效）
    setDOConfigLevel = [0, 0, 0, 0, 1, 1, 1, 1]
    getDOConfigLevel = [0] * 8
    rtn = robot.SetDOConfigLevel(setDOConfigLevel)
    print(f"SetDOConfigLevel rtn is {rtn}")
    rtn, getDOConfigLevel = robot.GetDOConfigLevel()
    print(f"GetDOConfigLevel rtn is {rtn}, value is {getDOConfigLevel[0]} {getDOConfigLevel[1]} {getDOConfigLevel[2]} {getDOConfigLevel[3]} {getDOConfigLevel[4]} {getDOConfigLevel[5]} {getDOConfigLevel[6]} {getDOConfigLevel[7]}")

    # 设置和获取工具DI电平配置
    setToolDIConfigLevel = [1, 0]
    getToolDIConfigLevel = [0] * 2
    rtn = robot.SetToolDIConfigLevel(setToolDIConfigLevel)
    print(f"SetToolDIConfigLevel rtn is {rtn}")
    rtn, getToolDIConfigLevel = robot.GetToolDIConfigLevel()
    print(f"GetToolDIConfigLevel rtn is {rtn}, value is {getToolDIConfigLevel[0]} {getToolDIConfigLevel[1]}")

    # 设置和获取标准DI电平配置
    setStandardDILevel = [1, 1, 1, 1, 0, 0, 0, 0]
    getStandardDILevel = [0] * 8
    rtn = robot.SetStandardDILevel(setStandardDILevel)
    print(f"SetStandardDILevel rtn is {rtn}")
    rtn, getStandardDILevel = robot.GetStandardDILevel()
    print(f"GetStandardDILevel rtn is {rtn}, value is {getStandardDILevel[0]} {getStandardDILevel[1]} {getStandardDILevel[2]} {getStandardDILevel[3]} {getStandardDILevel[4]} {getStandardDILevel[5]} {getStandardDILevel[6]} {getStandardDILevel[7]}")

    # 设置和获取标准DO电平配置
    setStandardDOLevel = [0, 0, 0, 0, 1, 1, 1, 1]
    getStandardDOLevel = [0] * 8
    rtn = robot.SetStandardDOLevel(setStandardDOLevel)
    print(f"SetStandardDOLevel rtn is {rtn}")
    rtn, getStandardDOLevel = robot.GetStandardDOLevel()
    print(f"GetStandsrdDOLevel rtn is {rtn}, value is {getStandardDOLevel[0]} {getStandardDOLevel[1]} {getStandardDOLevel[2]} {getStandardDOLevel[3]} {getStandardDOLevel[4]} {getStandardDOLevel[5]} {getStandardDOLevel[6]} {getStandardDOLevel[7]}")

    # 等待2秒
    time.sleep(2)

    # 关闭连接
    robot.CloseRPC()
    time.sleep(1)


# 调用测试函数
TestIOConfig(robot)
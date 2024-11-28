from fairino import Robot
import random
import time

# 与机器人控制器建立连接，连接成功返回一个机器人对象
robot = Robot.RPC('192.168.58.2')
robot.LoggerInit(output_model=0)
robot.SetLoggerLevel(4)

#上传末端Lua开放协议文件
error = robot.AxleLuaUpload("D://zUP/AXLE_LUA_End_DaHuan.lua")
print("AxleLuaUpload",error)

#设置末端通讯参数
error = robot.SetAxleCommunicationParam(7,8,1,0,5,3,1)
print("SetAxleCommunicationParam",error)

#获取末端通讯参数
error = robot.GetAxleCommunicationParam()
print("GetAxleCommunicationParam",error)

#设置启用末端LUA执行
error = robot.SetAxleLuaEnable(1)
print("SetAxleLuaEnable",error)

#获取末端LUA执行使能状态
error = robot.GetAxleLuaEnableStatus()
print("GetAxleLuaEnableStatus",error)

#设置末端LUA末端设备启用类型
error = robot.SetAxleLuaEnableDeviceType(0,1,0)
print("SetAxleLuaEnableDeviceType",error)

#获取末端LUA末端设备启用类型
error = robot.GetAxleLuaEnableDeviceType()
print("GetAxleLuaEnableDeviceType",error)

#设置启用夹爪动作控制功能
error = robot.SetAxleLuaGripperFunc(0,[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0])
print("SetAxleLuaGripperFunc",error)

#获取启用夹爪动作控制功能
error = robot.GetAxleLuaGripperFunc()
print("GetAxleLuaGripperFunc",error)

#获取当前配置的末端设备
error = robot.GetAxleLuaEnableDevice()
print("GetAxleLuaEnableDevice",error)

#激活夹爪
error = robot.ActGripper(1, 0)
print("ActGripper",error)
time.sleep(1)

#控制夹爪
error = robot.MoveGripper(1, 50, 100, 100, 5000, 0)
print("MoveGripper",error)
time.sleep(1)

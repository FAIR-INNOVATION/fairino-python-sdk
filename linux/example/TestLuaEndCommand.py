from fairino import Robot
import time
# A connection is established with the robot controller. A successful connection returns a robot object
robot = Robot.RPC('192.168.58.2')


def updateendlua(self):
    """上传末端Lua开放协议文件"""
    robot.LoggerInit(output_model=0)
    robot.SetLoggerLevel(lvl=4)
    #上传末端Lua开放协议文件(错误文件)
    # error = robot.AxleLuaUpload(filePath="D://zUP/AXLE_LUA_End_DaHuan_WeiHangBad.lua")
    # print("AxleLuaUpload return ",error)
    # 上传末端Lua开放协议文件(正确文件)
    error = robot.AxleLuaUpload(filePath="D://zUP/AXLE_LUA_End_DaHuan_WeiHang.lua")
    print("AxleLuaUpload return ", error)

    # # 设置末端通讯参数
    # error = robot.SetAxleCommunicationParam(7, 8, 1, 0, 5, 3, 1000)
    # print("SetAxleCommunicationParam", error)
    #
    # # 获取末端通讯参数
    # error,baudRate,dataBit,stopBit,verify,timeout,timeoutTimes,period = robot.GetAxleCommunicationParam()
    # print("GetAxleCommunicationParam return:", error)
    # print("末端通讯参数:", baudRate,dataBit,stopBit,verify,timeout,timeoutTimes,period)
    #
    # # 设置启用末端LUA执行
    # error = robot.SetAxleLuaEnable(1)
    # print("SetAxleLuaEnable return ", error)
    #
    # # 获取末端LUA执行使能状态
    # error,enable = robot.GetAxleLuaEnableStatus(enable=1)
    # print("GetAxleLuaEnableStatus return", enable)
    # print("末端LUA执行使能状态:", enable)
    #
    # # 设置末端LUA末端设备启用类型
    # error = robot.SetAxleLuaEnableDeviceType(0, 1, 0)
    # print("SetAxleLuaEnableDeviceType return ", error)
    #
    # # 获取末端LUA末端设备启用类型
    # error,forceSensorEnable,gripperEnable,IOEnable = robot.GetAxleLuaEnableDeviceType()
    # print("GetAxleLuaEnableDeviceType return ", error)
    # print("末端LUA末端设备启用类型:", forceSensorEnable,gripperEnable,IOEnable)

def endluagripper(self):
    """末端Lua开放协议夹爪功能"""
    # 设置末端通讯参数
    error = robot.SetAxleCommunicationParam(7, 8, 1, 0, 5, 3, 1000)
    print("SetAxleCommunicationParam", error)

    # 获取末端通讯参数
    error, baudRate, dataBit, stopBit, verify, timeout, timeoutTimes, period = robot.GetAxleCommunicationParam()
    print("GetAxleCommunicationParam return:", error)
    print("末端通讯参数:", baudRate, dataBit, stopBit, verify, timeout, timeoutTimes, period)

    # 设置启用末端LUA执行
    error = robot.SetAxleLuaEnable(1)
    print("SetAxleLuaEnable return ", error)

    # 获取末端LUA执行使能状态
    error, enable = robot.GetAxleLuaEnableStatus(enable=0)
    print("GetAxleLuaEnableStatus return", enable)
    print("末端LUA执行使能状态:", enable)

    # 设置末端LUA末端设备启用类型
    error = robot.SetAxleLuaEnableDeviceType(0, 1, 0)
    print("SetAxleLuaEnableDeviceType return", error)

    # 获取末端LUA末端设备启用类型
    error,forceSensorEnable,gripperEnable,IOEnable = robot.GetAxleLuaEnableDeviceType()
    print("GetAxleLuaEnableDeviceType return ", error)
    print("末端LUA末端设备启用类型:", forceSensorEnable,gripperEnable,IOEnable)


def endluaforce(self):
    """末端Lua开放协议力传感器功能"""
    # 设置末端通讯参数
    error = robot.SetAxleCommunicationParam(7, 8, 1, 0, 5, 3, 1000)
    print("SetAxleCommunicationParam", error)

    # 获取末端通讯参数
    error, baudRate, dataBit, stopBit, verify, timeout, timeoutTimes, period = robot.GetAxleCommunicationParam()
    print("GetAxleCommunicationParam return:", error)
    print("末端通讯参数:", baudRate, dataBit, stopBit, verify, timeout, timeoutTimes, period)

    # 设置启用末端LUA执行
    error = robot.SetAxleLuaEnable(1)
    print("SetAxleLuaEnable return ", error)

    # 获取末端LUA执行使能状态
    error, enable = robot.GetAxleLuaEnableStatus(enable=0)
    print("GetAxleLuaEnableStatus return", enable)
    print("末端LUA执行使能状态:", enable)

    # 设置末端LUA末端设备启用类型
    error = robot.SetAxleLuaEnableDeviceType(1, 0, 0)
    print("SetAxleLuaEnableDeviceType return", error)

    # 获取末端LUA末端设备启用类型
    error, forceSensorEnable, gripperEnable, IOEnable = robot.GetAxleLuaEnableDeviceType()
    print("GetAxleLuaEnableDeviceType return ", error)
    print("末端LUA末端设备启用类型:", forceSensorEnable, gripperEnable, IOEnable)


updateendlua(robot)
# endluagripper(robot)
# endluaforce(robot)
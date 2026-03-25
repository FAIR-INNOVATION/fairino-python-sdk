from time import sleep
import time
from fairino import Robot

# 与机器人控制器建立连接
robot = Robot.RPC('192.168.58.2')


def TestCtrlOpenLuaOperate(self):
    # 上传Lua文件到机器人
    rtn = robot.OpenLuaUpload("D://zUP/openlua/CtrlDev_WELDING_A.lua")
    print(f"OpenLuaUpload rtn is {rtn}")

    rtn = robot.OpenLuaUpload("D://zUP/openlua/CtrlDev_SWDPOLISH.lua")
    print(f"OpenLuaUpload rtn is {rtn}")

    # 从机器人下载Lua文件
    rtn = robot.OpenLuaDownload("CtrlDev_WELDING_A.lua", "D://zDOWN/")
    print(f"OpenLuaDownload rtn is {rtn}")

    rtn = robot.OpenLuaDownload("CtrlDev_SWDPOLISH.lua", "D://zDOWN/")
    print(f"OpenLuaDownload rtn is {rtn}")

    # 设置控制开放的Lua文件名
    rtn = robot.SetCtrlOpenLUAName(0, "CtrlDev_WELDING_A.lua")
    print(f"SetCtrlOpenLUAName rtn is {rtn}")

    rtn = robot.SetCtrlOpenLUAName(1, "CtrlDev_SWDPOLISH.lua")
    print(f"SetCtrlOpenLUAName rtn is {rtn}")

    # 获取控制开放的Lua文件名
    rtn, name = robot.GetCtrlOpenLUAName()
    print(f"ctrl open lua names : {name[0]}, {name[1]}, {name[2]}, {name[3]}")

    # 加载控制开放的Lua
    rtn = robot.LoadCtrlOpenLUA(1)
    print(f"LoadCtrlOpenLUA rtn is {rtn}")
    time.sleep(2)

    # 卸载控制开放的Lua
    rtn = robot.UnloadCtrlOpenLUA(1)
    print(f"UnloadCtrlOpenLUA rtn is {rtn}")

    # 删除指定的Lua文件
    rtn = robot.OpenLuaDelete("CtrlDev_WELDING_A.lua")
    print(f"OpenLuaDelete rtn is {rtn}")

    # 删除所有Lua文件
    rtn = robot.AllOpenLuaDelete()
    print(f"AllOpenLuaDelete rtn is {rtn}")

    # 关闭连接
    robot.CloseRPC()
    time.sleep(1)


# 调用测试函数
TestCtrlOpenLuaOperate(robot)
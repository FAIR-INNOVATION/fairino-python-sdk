from fairino import Robot
import time
# A connection is established with the robot controller. A successful connection returns a robot object
robot = Robot.RPC('192.168.58.2')

JP1 = [117.408,-86.777,81.499,-87.788,-92.964,92.959]
DP1 = [327.359,-420.973,518.377,-177.199,3.209,114.449]

JP2 = [72.515,-86.774,81.525,-87.724,-91.964,92.958]
DP2 = [-65.169,-529.17,518.018,-177.189,3.119,69.556]

# JP2_h = [72.515,-86.774,81.525,-87.724,-91.964,92.958]
DP2_h = [-65.169,-529.17,528.018,-177.189,3.119,69.556]

JP3 = [89.281,-102.959,81.527,-69.955,-86.755,92.958]
DP3 = [102.939,-378.069,613.165,176.687,1.217,86.329]

desc = [0,0,0,0,0,0]


def loaddefaultprogconfig(self):
    """设置开机自动加载程序"""
    # error = robot.ProgramLoad(program_name="/fruser/testPTP.lua")
    robot.LoadDefaultProgConfig(flag=0,program_name="/fruser/1010Test.lua")
    error,loadednamestr = robot.GetLoadedProgram()
    print("Loaded lua name is : ",loadednamestr)
    print("GetLoadedProgram return ",error)
    robot.ProgramRun()

def loaddefaultprogconfig1(self):
    """设置加载程序"""
    error = robot.ProgramLoad(program_name="/fruser/testPTP.lua")
    error,loadednamestr = robot.GetLoadedProgram()
    print("Loaded lua name is : ",loadednamestr)
    print("GetLoadedProgram return ",error)
    robot.ProgramRun()

def getprogramstate(self):
    """获取机器人程序运行状态和当前行号"""
    robot.Mode(state=0)
    robot.ProgramLoad(program_name="/fruser/1010Test.lua")
    error,loadednamestr = robot.GetLoadedProgram()
    print("Loaded lua name is : ", loadednamestr[0])
    robot.ProgramRun()
    while True:
        error,line = robot.GetCurrentLine()
        print("程序执行当前行号为：",line)
        error,state = robot.GetProgramState()
        print("程序执行状态为：", state)
        time.sleep(0.1)

def programrun(self):
    """控制机器人程序暂停、恢复、停止"""
    robot.Mode(state=0)
    robot.ProgramLoad(program_name="/fruser/1010Test.lua")
    error, loadednamestr = robot.GetLoadedProgram()
    print("Loaded lua name is : ", loadednamestr[0])
    robot.ProgramRun()
    time.sleep(1)

    for i in range(0,5):
        error = robot.PauseMotion()
        print("PauseMotion return ", error)
        time.sleep(2)
        error = robot.ResumeMotion()
        print("ResumeMotion return ", error)
        time.sleep(2)
    error = robot.StopMotion()
    print("StopMotion return ", error)

def luaupload(self):
    """上传LUA脚本"""
    error = robot.LuaUpload(filePath="D://zUP/1010TestLUA.lua")
    print("LuaUpload return ", error)

def luadownload(self):
    """下载LUA脚本"""
    error = robot.LuaDownLoad(fileName="1010TestLUA.lua",savePath="D://zUP/")
    print("LuaDownLoad return ", error)

def luadelete(self):
    """删除LUA脚本"""
    error = robot.LuaDelete(fileName="1010TestLUA.lua")
    print("LuaDelete return ", error)

def getlualist(self):
    """获取当前所有LUA文件名称"""
    error,lua_num,lua_name = robot.GetLuaList()
    print("Lua脚本数量：", lua_num)
    print("Lua脚本名称", lua_name)



# loaddefaultprogconfig(robot)
# loaddefaultprogconfig1(robot)
# getprogramstate(robot)
# programrun(robot)
# luaupload(robot)
# luadownload(robot)
# luadelete(robot)
getlualist(robot)

from fairino import Robot
import time

# A connection is established with the robot controller. A successful connection returns a robot object
robot = Robot.RPC('192.168.58.2')

robot.LuaDownLoad("test.lua", "/home/fr/test/download/")
time.sleep(1)

robot.LuaUpload("/home/fr/test/download/test1.lua")
time.sleep(1)

robot.LuaDelete("test2.lua")
time.sleep(1)

ret,num,name  = robot.GetLuaList()
print(num)
print(name)
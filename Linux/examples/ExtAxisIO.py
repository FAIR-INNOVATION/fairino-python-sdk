from fairino import Robot
# A connection is established with the robot controller. A successful connection returns a robot object
robot = Robot.RPC('192.168.58.2')
#Set extended DO
error = robot.SetAuxDO(1,True,False,True)
print("GetAuxAI",error)
#Set extended AO
error = robot.SetAuxAO(1,60,True)
print("SetAuxAO",error)
#Set the extended DI input filtering time
error = robot.SetAuxDIFilterTime(10)
print("SetAuxDIFilterTime",error)
#Set the extended AI input filtering time
error = robot.SetAuxAIFilterTime(0,10)
print("SetAuxAIFilterTime",error)
#Wait for the extended DI input
error = robot.WaitAuxDI(0,False,100,False)
print("WaitAuxDI",error)
#Wait for the extended AI input
error = robot.WaitAuxAI(0,0,100,500,False)
print("WaitAuxAI",error)
#Gets the extended AI value
error = robot.GetAuxAI(0,False)
print("GetAuxAI",error)
#Gets the extended DI value
error = robot.GetAuxDI(0,True)
print("GetAuxDI",error)

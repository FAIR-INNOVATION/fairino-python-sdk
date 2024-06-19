from fairino import Robot
# A connection is established with the robot controller. A successful connection returns a robot object
robot = Robot.RPC('192.168.58.2')

for i in range(0,16):
    error = robot.SetDO(i,1)      #Open the control box DO
error = robot.WaitMs(1000)
print("WaitMs:",error)
for i in range(0,16):
    robot.SetDO(i,0)      #Close the control box DO
robot.WaitMs(1000)

error_tooldo = 0
for i in range(0,2):
    error = robot.SetToolDO(i,1)     # Open the tool DO
robot.WaitMs(1000)
for i in range(0,2):
    error = robot.SetToolDO(i,0)     # Close the tool DO

error = robot.SetAO(0,100.0)
print("Set the AO0 of the control box", error)
error = robot.SetAO(1,100.0)
print("Set the AO1 of the control box ", error)

error = robot.SetToolAO(0,100.0)
print("Set tool analog output", error)
Robot.WaitMs(1000)
error = robot.SetToolAO(0,0.0)
print("Set tool analog output ", error)

error = robot.GetDI(0,0)
print("Obtain the di0 of the control box",error)

tool_di = robot.GetToolDI(1,0)
print("Obtain tool di1 ",tool_di)


error = robot.GetAI(0)
print("Obtain AI0",error)

error = robot.GetToolAI(0)
print("Obtain ToolAI0",error)


max_waittime = 2000
error = robot.WaitDI(0,1,max_waittime,0)
print("WaitDI ",error)
error = robot.WaitMultiDI(1,3,1,max_waittime,0)
print("WaitMultiDI",error)
error = robot.WaitToolDI(1,1,max_waittime,0)
print("WaitToolDI",error)


error = robot.WaitAI(0,0,50,max_waittime,1)
print("WaitAI ",error)
error = robot.WaitToolAI(0,0,50,max_waittime,0)
print("WaitToolAI ",error)




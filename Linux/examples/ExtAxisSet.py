from fairino import Robot
# A connection is established with the robot controller. A successful connection returns a robot object
robot = Robot.RPC('192.168.58.2')
#Configure UDP extension axis communication parameters
error = robot.ExtDevSetUDPComParam('192.168.58.88',2021,2,50,5,50,1,2,5)
print("ExtDevSetUDPComParam return:",error)
#Get the UDP extension axis communication parameter configuration
error = robot.ExtDevGetUDPComParam()
print("ExtDevGetUDPComParam return:",error)

#Load the UDP communication connection
error = robot.ExtDevLoadUDPDriver()
print("ExtDevLoadUDPDriver return:",error)
#Unmount the UDP communication connection
error = robot.ExtDevUnloadUDPDriver()
print("ExtDevUnloadUDPDriver return:",error)
#Load the UDP communication connection
error = robot.ExtDevLoadUDPDriver()
print("ExtDevLoadUDPDriver return:",error)

#The UDP extension axis communication is disconnected and the connection is restored
error = robot.ExtDevUDPClientComReset()
print("ExtDevUDPClientComReset return:",error)
#The UDP extension axis communication is disconnected abnormally
error = robot.ExtDevUDPClientComClose()
print("ExtDevUDPClientComClose return:",error)

#Set the position of the expansion robot relative to the expansion axis
error = robot.SetRobotPosToAxis(1)
print("SetRobotPosToAxis return:",error)
#Set the extended shaft system DH parameters
error = robot.SetAxisDHParaConfig(4,128.5,206.4,0,0,0,0,0,0,)
print("SetAxisDHParaConfig return:",error)
#Configure UDP extension axis parameters
error = robot.ExtAxisParamConfig(1,1,0,1000,-1000,1000,1000,1.905,262144,
                                 200,1,1,0)
print("ExtAxisParamConfig return:",error)




































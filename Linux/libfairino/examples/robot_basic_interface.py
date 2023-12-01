from fairino import Robot
import time

# A connection is established with the robot controller. A successful connection returns a robot object
robot = Robot.RPC('192.168.58.2')
print("Connect successfully，",robot)

ret,version  = robot.GetSDKVersion()    #Query SDK version numbe
if ret ==0:
    print("SDK version is", version )
else:
    print("The query failed with the error code is",ret)

ret,ip = robot.GetControllerIP()    #Obtain Controller IP
if ret ==0:
    print("controller ip is", ip)
else:
    print("the errcode is ",ret)

ret = robot.Mode(1) #The robot goes into manual mode
print("#The robot goes into manual mode", ret)
time.sleep(1)
ret = robot.DragTeachSwitch(1)  #When the robot enters the drag teaching mode, it can only enter the drag teaching mode in manual mode
print("the robot enters the drag teaching mode", ret)
time.sleep(1)
ret,state = robot.IsInDragTeach()    #Check whether the user is in drag mode, 1-Drag mode, 0-No drag mode)
if ret == 0:
    print("drag state is:", state)
else:
    print("the errcode is",ret)
time.sleep(3)
ret = robot.DragTeachSwitch(0)  #enter the non-drag teaching mode
print("enter the non-drag teaching mode", ret)
time.sleep(1)
ret,state = robot.IsInDragTeach()    #Check whether the user is in drag mode, 1-Drag mode, 0-No drag mode)
if ret == 0:
    print("drag state is：", state)
else:
    print("the errcode is：",ret)
time.sleep(3)

ret = robot.RobotEnable(0)   #Unable the robot
print("Unable the robot", ret)
time.sleep(3)
ret = robot.RobotEnable(1)   #This function is enabled on the robot. After the robot is powered on, it is automatically enabled by default
print("Enable the robot", ret)


ret = robot.Mode(0)   #The robot goes into automatic operation mode
print("The robot goes into automatic operation mode", ret)
time.sleep(1)
ret = robot.Mode(1)   #The robot goes into manual mode
print("The robot goes into manual mode", ret)

from fairino import Robot
import time
# A connection is established with the robot controller. A successful connection returns a robot object
robot = Robot.RPC('192.168.58.2')
error = robot.SetSpeed(20) # Set the global speed. Manual mode and automatic mode are set independently
print("Set global speed:",error)
for i in range(1,21):
    error = robot.SetSysVarValue(i,10)
robot.WaitMs(1000)
for i in range(1,21):
    sys_var = robot.GetSysVarValue(i)
    print("Variable number:",i," Variable value ",sys_var)


t_coord = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
robot.SetToolList(0,[0,0,0,0,0,0],0,0)
robot.SetWObjList(0,[0,0,0,0,0,0])
for i in range(1,7):
    robot.DragTeachSwitch(1)
    time.sleep(5)
    error = robot.SetToolPoint(i) # In fact, the robot should be controlled to move to the appropriate position according to the requirements before sending the command
    print("Set tool reference point - six point method",i,"errcode",error)
    robot.DragTeachSwitch(0)
    time.sleep(1)
error = robot.ComputeTool()
print("Calculation tool coordinate system - six point method ",error)
for i in range(1,5):
    robot.DragTeachSwitch(1)
    time.sleep(5)
    error = robot.SetTcp4RefPoint(i) # In fact, the robot should be controlled to move to the appropriate position according to the requirements before sending the command
    print("Set tool reference point - four point method",i,"errcode",error)
    robot.DragTeachSwitch(0)
    time.sleep(1)
error,t_coord= robot.ComputeTcp4()
print("Calculation tool coordinate system - six point method ",error,"tool TCP",t_coord)
error = robot.SetToolCoord(10,t_coord,0,0)
print("Set Tool Coordinate System",error)

error = robot.SetToolList(10,t_coord,0,0)
print("Set Tool Coordinate Series Table",error)

etcp = [1.0,2.0,3.0,4.0,5.0,6.0]
etool = [21.0,22.0,23.0,24.0,25.0,26.0]
robot.SetToolList(0,[0,0,0,0,0,0],0,0)
robot.SetWObjList(0,[0,0,0,0,0,0])
for i in range(1,4):
    error = robot.SetExTCPPoint(i) # In fact, the robot should be controlled to move to the appropriate position according to the requirements before sending the command
    print("Set external tool reference point - three point method，point:",i,"errcode",error)
    time.sleep(1)
error,etcp = robot.ComputeExTCF()
print("Calculation external tool coordinate system - three point method, errcode ",error," etcp: ",etcp)
etool = [21.0,22.0,23.0,24.0,25.0,26.0]
error = robot.SetExToolCoord(10,etcp,etool)
print("Set the external tool coordinate system ",error)
error = robot.SetExToolList(10,etcp,etool)
print("Set external tool coordinate series table",error)

w_coord = [11.0,12.0,13.0,14.0,15.0,16.0]
robot.SetToolList(0,[0,0,0,0,0,0],0,0)
robot.SetWObjList(0,[0,0,0,0,0,0])
for i in range(1,4):
    error = robot.SetWObjCoordPoint(i) # In fact, the robot should be controlled to move to the appropriate position according to the requirements before sending the command
    print("Set the workpiece reference point - three point method，point",i,"errcode",error)
    time.sleep(1)
error, w_coord = robot.ComputeWObjCoord(0)
print("Calculation of workpiece coordinate system - three point method,errcode: ",error," workpiece coordinate syste:", w_coord)
error = robot.SetWObjCoord(11,w_coord)
print("Set the workpiece coordinate system ",error)
error = robot.SetWObjList(11,w_coord)
print("Set the workpiece coordinate series table ",error)

error = robot.SetLoadWeight(0)#！The load weight setting should match the actual load weight setting (wrong load weight setting may cause the robot to lose control in drag mode)
error = robot.SetLoadCoord(3.0,4.0,5.0) #！The load centroid setting should match the actual setting (wrong load centroid setting may cause the robot to lose control in drag mode)
print("Set the end load centroid coordinates ",error)

error = robot.SetRobotInstallPos(0) #！！！The setting of the installation mode should be consistent with the actual setting (the wrong setting of the installation mode will cause the robot to lose control in drag mode)
print("Set the robot installation method ",error)

error = robot.SetRobotInstallAngle(0.0,0.0) #！！！The installation Angle setting should be consistent with the actual setting (the wrong installation Angle setting will cause the robot to lose control in drag mode)
print("Set robot installation angle - free installation ",error)


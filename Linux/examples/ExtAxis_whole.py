from fairino import Robot
# A connection is established with the robot controller. A successful connection returns a robot object
robot = Robot.RPC('192.168.58.2')
#1.Calibrate and apply the robot tool coordinate system. You can use the four-point method or the six-point method
# to calibrate and apply the tool coordinate system. The interfaces involved in the calibration of the
# tool coordinate system are as follows：

point_num=1
id=1
coord=[100,200,300,0,0,0,]
type=0
install=0
#1.设置工具坐标系
# robot.SetToolPoint(point_num)  #Set tool reference point - six point method
# robot.ComputeTool() #Computational tool coordinate system
# robot.SetTcp4RefPoint()   #Set tool reference point - four point method
# robot.ComputeTcp4()   #Calculating tool coordinate system - four-point method
# robot.SetToolCoord(id, coord,type,install)  #Set the application tool coordinate system
# robot.SetToolList(id, coord,type,install)   #Sets the list of application tool coordinate systems

#2.Set UDP communication parameters and load UDP communication
robot.ExtDevSetUDPComParam("192.168.58.88", 2021, 2, 100, 3, 100, 1, 100, 10);
robot.ExtDevLoadUDPDriver();

#3.Set the extension shaft parameters, including the extension shaft type, extension shaft drive parameters, and extension shaft DH parameters
robot.SetAxisDHParaConfig(4, 200, 200, 0, 0, 0, 0, 0, 0)#Single axis positioner and DH parameters
robot.SetRobotPosToAxis(1);  #Expansion shaft mounting position
robot.ExtAxisParamConfig(1, 0, 1, 100, -100, 10, 10, 12, 131072, 0, 1, 0, 0)#Servo drive parameters, this example is a single-axis positioner, so only one drive parameter needs to be set. If you choose an extension shaft type with multiple axes, you need to set the drive parameters for each axis

#4.Set the selected axis to enable and homing
robot.ExtAxisServoOn(1, 0);
robot.ExtAxisSetHoming(1, 0, 20, 3);

#5.Carry out calibration and application of extended axis coordinate system
pos =[0,0,0,0,0,0] #Enter your marker coordinates
robot.SetRefPointInExAxisEnd(pos)
robot.PositionorSetRefPoint(1)#You need to calibrate the extension axis through four points in different locations, so you need to call this interface four times to complete the calibration
error,coord = robot.PositionorComputeECoordSys()#Calculate the calibration results of the extension axis
robot.ExtAxisActiveECoordSys(1, 1, coord, 1); #The calibration results are applied to the extended axis coordinate system

method=1
#6.To calibrate the workpiece coordinate system on the extension axis, you need the following interfaces
# robot.SetWObjCoordPoint( point_num)
# error,coord=robot.ComputeWObjCoord( method)
# robot.SetWObjCoord(id,coord)
# robot.SetWObjList(id, coord)

#7.Record the start point of your synchronous joint movement
startdescPose = [0,0,0,0,0,0]#Enter your coordinates
startjointPos = [0,0,0,0,0,0]#Enter your coordinates
startexaxisPos = [0,0,0,0,]#Enter your coordinates

#8.Record the coordinates of the end point of your synchronous joint movement
enddescPose = [0,0,0,0,0,0]#Enter your coordinates
endjointPos = [0,0,0,0,0,0]#Enter your coordinates
endexaxisPos = [0,0,0,0,]#Enter your coordinates


#9.Write synchronous motion program
#Move to the starting point, assuming that the tool coordinate system and the workpiece coordinate system are both 1
robot.ExtAxisMove(startexaxisPos, 20);
robot.MoveJ(startjointPos,  1, 1, desc_pos=startdescPose,exaxis_pos=startexaxisPos);

#Start synchronized motion
robot.ExtAxisSyncMoveJ(endjointPos, enddescPose, 1, 1, endexaxisPos);
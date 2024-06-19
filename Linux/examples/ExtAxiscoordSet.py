import time
from fairino import Robot
# A connection is established with the robot controller. A successful connection returns a robot object
robot = Robot.RPC('192.168.58.2')
#Set the reference point of the extended axis coordinate system - four-point method
error = robot.ExtAxisSetRefPoint(1)
print("ExtAxisComputeECoordSys(1) return:",error)
time.sleep(5)
error = robot.ExtAxisSetRefPoint(2)
print("ExtAxisComputeECoordSys(2) return:",error)
time.sleep(5)
error = robot.ExtAxisSetRefPoint(3)
print("ExtAxisComputeECoordSys(3) return:",error)
time.sleep(5)
error = robot.ExtAxisSetRefPoint(4)
print("ExtAxisComputeECoordSys(4) return:",error)
#Calculation of extended axis coordinate system - four-point method
error,coord = robot.ExtAxisComputeECoordSys()
print("ExtAxisComputeECoordSys() return:",error,coord)
#Apply the extended axis coordinate system
error = robot.ExtAxisActiveECoordSys(1,1,coord,1)
print("ExtAxisActiveECoordSys() return:",error)

error,desc_pos = robot.GetActualTCPPose()
print("GetActualTCPPose",error,desc_pos)
#Set the pose of the calibration reference point in the end coordinate system of the positioner
error = robot.SetRefPointInExAxisEnd(desc_pos)
print("SetRefPointInExAxisEnd(1) return:",error)

#Positioner coordinate system reference point setting - four-point method
error = robot.PositionorSetRefPoint(1)
print("PositionorSetRefPoint(1) return:",error)
time.sleep(15)
error = robot.ExtAxisStartJog(1,1,20,20,5)
print("ExtAxisStartJog return:",error)
error = robot.ExtAxisStartJog(2,1,20,20,5)
print("ExtAxisStartJog return:",error)
time.sleep(10)
error = robot.PositionorSetRefPoint(2)
print("PositionorSetRefPoint(2) return:",error)
time.sleep(5)
error = robot.ExtAxisStartJog(1,1,20,20,5)
print("ExtAxisStartJog return:",error)
error = robot.ExtAxisStartJog(2,1,20,20,5)
print("ExtAxisStartJog return:",error)
time.sleep(10)
error = robot.PositionorSetRefPoint(3)
print("PositionorSetRefPoint(3) return:",error)
time.sleep(5)
error = robot.ExtAxisStartJog(1,1,20,20,5)
print("ExtAxisStartJog return:",error)
error = robot.ExtAxisStartJog(2,1,20,20,5)
print("ExtAxisStartJog return:",error)
time.sleep(10)
error = robot.PositionorSetRefPoint(4)
print("PositionorSetRefPoint(4) return:",error)
#Coordinate system calculation of positioner - four-point method
error,coord = robot.PositionorComputeECoordSys()
print("PositionorComputeECoordSys() return:",error,coord)



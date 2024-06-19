from fairino import Robot
import time
# A connection is established with the robot controller. A successful connection returns a robot object
robot = Robot.RPC('192.168.58.2')

error = robot.FT_SetRCS(0)    # Set the force sensor reference coordinate system
print(' Set the force sensor reference coordinate system ',error)
time.sleep(1)

tool_id = 10  #Sensor coordinate number
tool_coord = [0.0,0.0,35.0,0.0,0.0,0.0]   # Position of sensor relative to end flange
tool_type = 1  # 0-Tool, 1-Sensor
tool_install = 0 # 0-Mount end, 1-Outside of robot
robot.SetToolCoord(tool_id,tool_coord,tool_type,tool_install)     #Set sensor coordinate system, sensor relative end flange position
time.sleep(1)
error = robot.FT_PdIdenRecord(tool_id)   # Load weight identification record
print('Load weight identification record ',error)
time.sleep(1)
error = robot.FT_PdIdenRecord()  # Load weight identification calculation
print('Load weight identification calculation ',error)

#For load centroid identification, the robot needs to teach three different poses, then record the identification data, and finally calculate the load centroid
robot.Mode(1)
ret = robot.DragTeachSwitch(1)
time.sleep(5)
ret = robot.DragTeachSwitch(0)
time.sleep(1)
error = robot.FT_PdCogIdenRecord(tool_id,1)
print(' Load centroid identification record ',error)
ret = robot.DragTeachSwitch(1)
time.sleep(5)
ret = robot.DragTeachSwitch(0)
time.sleep(1)
error = robot.FT_PdCogIdenRecord(tool_id,2)
print('Load centroid identification record ',error)
ret = robot.DragTeachSwitch(1)
time.sleep(5)
ret = robot.DragTeachSwitch(0)
time.sleep(1)
error = robot.FT_PdCogIdenRecord(tool_id,3)
print(' Load centroid identification record ',error)
time.sleep(1)
error = robot.FT_PdCogIdenCompute()
print('Load centroid identification calculation ',error)




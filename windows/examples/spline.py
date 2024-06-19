from fairino import Robot
# A connection is established with the robot controller. A successful connection returns a robot object
robot = Robot.RPC('192.168.58.2')

tool = 0 #Tool number
user = 0 #Workpiece number

lastFlag= 0 # Is it the last point, 0-No, 1-Yes;

joint_pos1 = [116.489,-85.278,111.501,-112.486,-85.561,24.693]
joint_pos2 = [86.489,-65.278,101.501,-112.486,-85.561,24.693]
joint_pos3 = [116.489,-45.278,91.501,-82.486,-85.561,24.693]

desc_pos4 = [236.794,-375.119, 65.379, -176.938, 2.535, -179.829]
desc_pos5 = [236.794,-275.119, 165.379, -176.938, 2.535, -179.829]
desc_pos6 = [286.794,-375.119, 265.379, -176.938, 2.535, -179.829]

ret = robot.SplineStart() #	Spline motion start
print(" Spline motion start: errcode ", ret)
ret = robot.SplinePTP(joint_pos1, tool, user)   # Spline motion PTP
print("Spline motion PTP: errcode ", ret)
ret = robot.SplinePTP(joint_pos2, tool, user)   # Spline motion PTP
print("Spline motion PTP: errcode ", ret)
ret = robot.SplinePTP(joint_pos3, tool, user)   # Spline motion PTP
print("Spline motion PTP: errcode ", ret)
ret = robot.SplineEnd() # Spline motion end
print("Spline motion end", ret)

ret = robot.NewSplineStart(1) # New spline motion start
print("New spline motion start:errcode", ret)
ret = robot.NewSplinePoint(desc_pos4, tool, user, lastFlag)# New Spline Instruction Points
print("New Spline Instruction Points:errcode ", ret)
ret = robot.NewSplinePoint(desc_pos5, tool, user, lastFlag, vel=30)# New Spline Instruction Points
print("New Spline Instruction Points:errcode ", ret)
lastFlag = 1
ret = robot.NewSplinePoint(desc_pos6, tool, user, lastFlag, vel=30)# New Spline Instruction Points
print("New Spline Instruction Points:errcode ", ret)
ret = robot.NewSplineEnd() # New spline motion end
print("New spline motion end:errcode ", ret)

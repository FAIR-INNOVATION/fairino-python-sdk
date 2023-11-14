from fairino import Robot
# A connection is established with the robot controller. A successful connection returns a robot object
robot = Robot.RPC('192.168.58.2')
ret = robot.GetRobotInstallAngle()
print("Obtain robot installation angle", ret)
for i in range(1,21):
    error = robot.GetSysVarValue(i)
    print("Obtain system variable number:",i,"value:", error)
ret = robot.GetActualJointPosDegree()
print("Obtain the current joint position (angle)", ret)
ret = robot.GetActualJointPosRadian()
print("Obtain the current joint position(radian)", ret)
ret = robot.GetActualJointSpeedsDegree()
print("Obtain joint Actual Speed -deg/s ", ret)
ret = robot.GetTargetTCPCompositeSpeed()
print("Obtain Target TCP Composite Speed", ret)
ret = robot.GetActualTCPCompositeSpeed()
print("Obtain Actual TCP Composite Speed ", ret)
ret = robot.GetTargetTCPSpeed()
print("Obtain Target TCP Speed ", ret)
ret = robot.GetActualTCPSpeed()
print("Obtain Actual TCP Speed ", ret)
ret = robot.GetActualTCPPose()
print("Obtain the current tool pose", ret)
ret = robot.GetActualTCPNum()
print("Obtain the current tool coordinate system number", ret)
ret = robot.GetActualWObjNum()
print("Obtain the current workpiece coordinate system number", ret)
ret = robot.GetActualToolFlangePose()
print("Obtain the current end flange pose ", ret)
ret = robot.GetJointTorques()
print("Obtain the current joint torque ", ret)
ret = robot.GetTargetPayload(0)
print("Obtain the weight of the current load ", ret)
ret = robot.GetTargetPayloadCog(0)
print("Obtain the centroid of the current load", ret)
ret = robot.GetRobotCurJointsConfig()
print("Obtain the current joint configuration of the robot ", ret)

ret = robot.GetSystemClock()
print("Get system time", ret)
ret = robot.GetDefaultTransVel()
print("Obtain default speed ", ret)

ret = robot.GetTCPOffset()
print("Obtain the current tool coordinate system", ret)

ret = robot.GetWObjOffset()
print("Obtain the current workpiece coordinate system", ret)
ret = robot.GetJointSoftLimitDeg()
print("Obtain joint soft limit angle", ret)
ret = robot.GetRobotMotionDone()
print("Check if the robot motion is complete ", ret)
ret = robot.GetRobotErrorCode()
print("Obtain the robot error code ", ret)
ret = robot.GetRobotTeachingPoint("11")
print("Obtain the robot teaching point data", ret)


J1=[90.442,-100.149,-90.699,-60.347,90.580,-47.174]
P1=[70.414,368.526,338.135,-178.348,-0.930,52.611]
ret = robot.GetInverseKin(0,P1,config=-1)
print("Inverse kinematics, Cartesian pose to solve joint position ", ret)
ret = robot.GetInverseKinRef(0,P1,J1)
print("Inverse kinematics solve inverse kinematics, tool pose solve joint position, and refer to specified joint position to solve", ret)
ret = robot.GetInverseKinHasSolution(0,P1,J1)
print("Inverse kinematics, tool pose solution, whether joint position is solved", ret)
ret = robot.GetForwardKin(J1)
print("Forward kinematics, joint position solving tool pose", ret)

ret = robot.GetSSHKeygen() # Obtain SSH Keygen
print("Obtain SSH Keygen", ret)

ret = robot.ComputeFileMD5("/fruser/201.lua")   #计算指定路径下文件的MD5值
print("Calculates the MD5 value of the file in the specified path ", ret)


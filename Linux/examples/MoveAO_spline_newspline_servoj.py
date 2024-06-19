from fairino import Robot
import time
# A connection is established with the robot controller. A successful connection returns a robot object
robot = Robot.RPC('192.168.58.2')
tool =1
user =1
#Control box movement AO starts
error = robot.MoveAOStart(0,100,98,1)
print("MoveAOStart",error)
error,joint_pos = robot.GetActualJointPosDegree()
print("GetActualJointPosDegree",error,joint_pos)
joint_pos[0] = joint_pos[0]+10
#The spline movement begins
error = robot.SplineStart()
print("SplineStart",error)
error = robot.SplinePTP(joint_pos,tool,tool)   #The spline movement PTP
print("SplinePTP",error)
joint_pos[0] = joint_pos[0]-10
error = robot.SplinePTP(joint_pos, tool, user)   #The spline movement PTP
print("SplinePTP",error)
error = robot.SplineEnd() #The spline movement end
print("SplineEnd",error)
#Control box movement AO stops
error = robot.MoveAOStop()
print("MoveAOStop",error)


#Tool movement AO stars
error = robot.MoveToolAOStart(0,100,100,1)
print("MoveToolAOStart",error)
error,joint_pos = robot.GetActualJointPosDegree()
print("GetActualJointPosDegree",error,joint_pos)
joint_pos[0] = joint_pos[0]+10
#The spline movement begins
error = robot.SplineStart()
print("SplineStart",error)
error = robot.SplinePTP(joint_pos,tool,tool)   #The spline movement PTP
print("SplinePTP",error)
joint_pos[0] = joint_pos[0]-10
error = robot.SplinePTP(joint_pos, tool, user)   #The spline movement PTP
print("SplinePTP",error)
error = robot.SplineEnd() #The spline movement end
print("SplineEnd",error)
#Tool movement AO stops
error = robot.MoveToolAOStop()
print("MoveToolAOStop",error)



#Control box movement AO starts
error = robot.MoveAOStart(0,100,100,1)
print("MoveAOStart",error)
error,desc_pos = robot.GetActualTCPPose()
print("GetActualTCPPose",error,desc_pos)
desc_pos1 =[desc_pos[0],desc_pos[1],desc_pos[2],desc_pos[3],desc_pos[4],desc_pos[5]]
desc_pos2 =[desc_pos[0],desc_pos[1],desc_pos[2],desc_pos[3],desc_pos[4],desc_pos[5]]
desc_pos3 =[desc_pos[0],desc_pos[1],desc_pos[2],desc_pos[3],desc_pos[4],desc_pos[5]]
desc_pos4 =[desc_pos[0],desc_pos[1],desc_pos[2],desc_pos[3],desc_pos[4],desc_pos[5]]
desc_pos1[1] = desc_pos1[1]-50
desc_pos1[2] = desc_pos1[2]-50
desc_pos2[1] = desc_pos2[1]-100
desc_pos2[2] = desc_pos2[2]+50
desc_pos3[1] = desc_pos3[1]-150
desc_pos3[2] = desc_pos3[2]-50
desc_pos4[1] = desc_pos4[1]-200
desc_pos4[2] = desc_pos4[2]+50
error = robot.MoveL(desc_pos1,1,1)
print("MoveL",error)
#新The spline movement begins
lastFlag = 0
error = robot.NewSplineStart(1)
print("NewSplineStart",error)
print("desc_pos",desc_pos)
error = robot.NewSplinePoint(desc_pos1, tool, user, lastFlag)#The new spline movement point
print("NewSplinePoint",error)
error = robot.NewSplinePoint(desc_pos2, tool, user, lastFlag, vel=30)#The new spline movement point
print("NewSplinePoint",error)
error = robot.NewSplinePoint(desc_pos3, tool, user, lastFlag, vel=30)#The new spline movement point
print("NewSplinePoint",error)
lastFlag = 1
error = robot.NewSplinePoint(desc_pos4, tool, user, lastFlag, vel=30)#The new spline movement point
print("NewSplinePoint",error)
error = robot.NewSplineEnd() #The new spline movement end
print("NewSplineEnd",error)
#Control box movement AO stops
error = robot.MoveAOStop()
print("MoveAOStop",error)


#Tool movement AO stars
error = robot.MoveToolAOStart(0,100,98,1)
print("MoveToolAOStart",error)
error,desc_pos = robot.GetActualTCPPose()
print("GetActualTCPPose",error,desc_pos)
desc_pos1 =[desc_pos[0],desc_pos[1],desc_pos[2],desc_pos[3],desc_pos[4],desc_pos[5]]
desc_pos2 =[desc_pos[0],desc_pos[1],desc_pos[2],desc_pos[3],desc_pos[4],desc_pos[5]]
desc_pos3 =[desc_pos[0],desc_pos[1],desc_pos[2],desc_pos[3],desc_pos[4],desc_pos[5]]
desc_pos4 =[desc_pos[0],desc_pos[1],desc_pos[2],desc_pos[3],desc_pos[4],desc_pos[5]]
desc_pos1[1] = desc_pos1[1]+50
desc_pos1[2] = desc_pos1[2]-50
desc_pos2[1] = desc_pos2[1]+100
desc_pos2[2] = desc_pos2[2]+50
desc_pos3[1] = desc_pos3[1]+150
desc_pos3[2] = desc_pos3[2]-50
desc_pos4[1] = desc_pos4[1]+200
desc_pos4[2] = desc_pos4[2]+50
error = robot.MoveL(desc_pos1,1,1)
print("MoveL",error)
#The new spline movement begins
lastFlag = 0
error = robot.NewSplineStart(1)
print("NewSplineStart",error)
print("desc_pos",desc_pos)
error = robot.NewSplinePoint(desc_pos1, tool, user, lastFlag)#The new spline movement point
print("NewSplinePoint",error)
error = robot.NewSplinePoint(desc_pos2, tool, user, lastFlag, vel=30)#The new spline movement point
print("NewSplinePoint",error)
error = robot.NewSplinePoint(desc_pos3, tool, user, lastFlag, vel=30)#The new spline movement point
print("NewSplinePoint",error)
lastFlag = 1
error = robot.NewSplinePoint(desc_pos4, tool, user, lastFlag, vel=30)#The new spline movement point
print("NewSplinePoint",error)
error = robot.NewSplineEnd() #The new spline movement end
print("NewSplineEnd",error)
#Tool movement AO stops
error = robot.MoveToolAOStop()
print("MoveToolAOStop",error)


#Control box movement AO starts
error = robot.MoveAOStart(0,100,98,1)
print("MoveAOStart",error)
error,joint_pos = robot.GetActualJointPosDegree()
print("GetActualJointPosDegree",joint_pos)
error_joint = 0
count =100
error = robot.ServoMoveStart()  #Start of servo motion
print("ServoMoveStart",error)
while(count):
    error = robot.ServoJ(joint_pos)   #Joint space servo mode motion
    if error!=0:
        error_joint =error
    joint_pos[0] = joint_pos[0] + 0.1  #1 axis movement 0.1 degrees each time, movement 100 times
    count = count - 1
    time.sleep(0.008)
print("ServoJ",error_joint)
error = robot.ServoMoveEnd()  #servo mode motion end
print("ServoMoveEnd",error)
#Control box movement AO stops
error = robot.MoveAOStop()
print("MoveAOStop",error)

mode = 2  #[0]- absolute motion (base coordinates), [1]- incremental motion (base coordinates), [2]- incremental motion (tool coordinates)
n_pos = [0.0,0.0,0.5,0.0,0.0,0.0]   # Cartesian space pose increment
error,desc_pos = robot.GetActualTCPPose()
print("GetActualTCPPose",desc_pos)
count = 100
error_cart =0
error = robot.ServoMoveStart()  #Start of servo motion
print("ServoMoveStart",error)
while(count):
    error = robot.ServoCart(mode, n_pos, vel=40)   #Descartes space servo mode motion
    if error!=0:
        error_cart =error
    count = count - 1
    time.sleep(0.008)
print("笛卡尔空间伺服模式运动错误码", error_cart)
error = robot.ServoMoveEnd()  #End of servo motion
print("伺服运动end错误码",error)
time.sleep(3)

#Tool movement AO stars
error = robot.MoveToolAOStart(0,100,98,1)
print("MoveToolAOStart",error)
mode = 2  #[0]- absolute motion (base coordinates), [1]- incremental motion (base coordinates), [2]- incremental motion (tool coordinates)
n_pos = [0.0,0.0,-0.5,0.0,0.0,0.0]   # Cartesian space pose increment
error,desc_pos = robot.GetActualTCPPose()
print("GetActualTCPPose",desc_pos)
count = 100
error_cart =0
error = robot.ServoMoveStart()  #Start of servo motion
print("ServoMoveStart",error)
while(count):
    error = robot.ServoCart(mode, n_pos, vel=40)   #Descartes space servo mode motion
    if error!=0:
        error_cart =error
    count = count - 1
    time.sleep(0.008)
print("ServoCart", error_cart)
error = robot.ServoMoveEnd()  #End of servo motion
print("ServoMoveEnd",error)
#Tool movement AO stops
error = robot.MoveToolAOStop()
print("MoveToolAOStop",error)



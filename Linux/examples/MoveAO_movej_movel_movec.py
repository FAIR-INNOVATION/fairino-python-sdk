from fairino import Robot
import time
# A connection is established with the robot controller. A successful connection returns a robot object
robot = Robot.RPC('192.168.58.2')

#Control box movement AO starts
error = robot.MoveAOStart(0,100,98,1)
print("MoveAOStart",error)
error,joint_pos = robot.GetActualJointPosDegree()
print("GetActualJointPosDegree",error,joint_pos)
joint_pos[0] = joint_pos[0]+10
#Robot joint motion
error = robot.MoveJ(joint_pos,1,1)
print("MoveJ",error)
time.sleep(3)
#Control box movement AO stops
error = robot.MoveAOStop()
print("MoveAOStop",error)


#Tool movement AO stars
error = robot.MoveToolAOStart(0,100,98,1)
print("MoveToolAOStart",error)
error,joint_pos = robot.GetActualJointPosDegree()
print("GetActualJointPosDegree",error,joint_pos)
joint_pos[0] = joint_pos[0]-10
#Robot joint motion
error = robot.MoveJ(joint_pos,1,1)
print("MoveJ",error)
time.sleep(3)
#Tool movement AO stops
error = robot.MoveToolAOStop()
print("MoveToolAOStop",error)


#Control box movement AO starts
error = robot.MoveAOStart(0,100,98,1)
print("MoveAOStart",error)
error,desc_pos = robot.GetActualTCPPose()
print("GetActualTCPPose",error,desc_pos)
desc_pos[2] = desc_pos[2]+50
#Rectilinear motion in Cartesian space
error = robot.MoveL(desc_pos,1,1)
print("MoveL",error)
time.sleep(3)
#Control box movement AO stops
error = robot.MoveAOStop()
print("MoveAOStop",error)


#Tool movement AO stars
error = robot.MoveToolAOStart(0,100,98,1)
print("MoveToolAOStart",error)
error,desc_pos = robot.GetActualTCPPose()
print("GetActualTCPPose",error,desc_pos)
desc_pos[2] = desc_pos[2]-50
#Rectilinear motion in Cartesian space
error = robot.MoveL(desc_pos,1,1)
print("MoveL",error)
time.sleep(3)
#Tool movement AO stops
error = robot.MoveToolAOStop()
print("MoveToolAOStop",error)



#Control box movement AO starts
error = robot.MoveAOStart(0,100,98,1)
print("MoveAOStart",error)
error,desc_pos = robot.GetActualTCPPose()
print("GetActualTCPPose",error,desc_pos)
desc_pos_mid =[desc_pos[0],desc_pos[1],desc_pos[2],desc_pos[3],desc_pos[4],desc_pos[5]]
desc_pos_end =[desc_pos[0],desc_pos[1],desc_pos[2],desc_pos[3],desc_pos[4],desc_pos[5]]
desc_pos_mid[0] = desc_pos_mid[0]+20
desc_pos_mid[1] = desc_pos_mid[1]-20
desc_pos_end[0]=desc_pos_end[0]+40
desc_pos_end[1]=desc_pos_end[1]
#Circular motion in Cartesian space
error = robot.MoveC(desc_pos_mid,1,1,desc_pos_end,1,1)
print("MoveC",error)
#Control box movement AO stops
error = robot.MoveAOStop()
print("MoveAOStop",error)

time.sleep((1))
#Tool movement AO stars
error = robot.MoveToolAOStart(0,100,98,1)
print("MoveToolAOStart",error)
error,desc_pos = robot.GetActualTCPPose()
print("GetActualTCPPose",error,desc_pos)
desc_pos_mid =[desc_pos[0],desc_pos[1],desc_pos[2],desc_pos[3],desc_pos[4],desc_pos[5]]
desc_pos_end =[desc_pos[0],desc_pos[1],desc_pos[2],desc_pos[3],desc_pos[4],desc_pos[5]]
desc_pos_mid[0] = desc_pos_mid[0]-20
desc_pos_mid[1] = desc_pos_mid[1]+20
desc_pos_end[0]=desc_pos_end[0]-40
desc_pos_end[1]=desc_pos_end[1]
#Circular motion in Cartesian space
error = robot.MoveC(desc_pos_mid,1,1,desc_pos_end,1,1)
print("MoveC",error)
time.sleep(3)
time.sleep(3)
#Tool movement AO stops
error = robot.MoveToolAOStop()
print("MoveToolAOStop",error)

#Control box movement AO starts
error = robot.MoveAOStart(0,100,98,1)
print("MoveAOStart",error)
error,desc_pos = robot.GetActualTCPPose()
print("GetActualTCPPose",error,desc_pos)
desc_pos_mid =[desc_pos[0],desc_pos[1],desc_pos[2],desc_pos[3],desc_pos[4],desc_pos[5]]
desc_pos_end =[desc_pos[0],desc_pos[1],desc_pos[2],desc_pos[3],desc_pos[4],desc_pos[5]]
desc_pos_mid[0] = desc_pos_mid[0]-20
desc_pos_mid[1] = desc_pos_mid[1]+20
desc_pos_end[0]=desc_pos_end[0]-40
desc_pos_end[1]=desc_pos_end[1]
#Circular motion in Cartesian space
error = robot.Circle(desc_pos_mid,1,1,desc_pos_end,1,1)
print("Circle",error)
time.sleep(3)
#Control box movement AO stops
error = robot.MoveAOStop()
print("MoveAOStop",error)


#Tool movement AO stars
error = robot.MoveToolAOStart(0,100,98,1)
print("MoveToolAOStart",error)
error,desc_pos = robot.GetActualTCPPose()
print("GetActualTCPPose",error,desc_pos)
desc_pos_mid =[desc_pos[0],desc_pos[1],desc_pos[2],desc_pos[3],desc_pos[4],desc_pos[5]]
desc_pos_end =[desc_pos[0],desc_pos[1],desc_pos[2],desc_pos[3],desc_pos[4],desc_pos[5]]
desc_pos_mid[0] = desc_pos_mid[0]+20
desc_pos_mid[1] = desc_pos_mid[1]-20
desc_pos_end[0]=desc_pos_end[0]+40
desc_pos_end[1]=desc_pos_end[1]
#Circular motion in Cartesian space
error = robot.Circle(desc_pos_mid,1,1,desc_pos_end,1,1)
print("Circle",error)
time.sleep(3)
time.sleep(3)
#Tool movement AO stops
error = robot.MoveToolAOStop()
print("MoveToolAOStop",error)



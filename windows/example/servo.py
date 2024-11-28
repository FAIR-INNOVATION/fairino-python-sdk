from fairino import Robot
import time
# A connection is established with the robot controller. A successful connection returns a robot object
robot = Robot.RPC('192.168.58.2')

error,joint_pos = robot.GetActualJointPosDegree()
print("Current joint position of robot :",joint_pos)
joint_pos = [joint_pos[0],joint_pos[1],joint_pos[2],joint_pos[3],joint_pos[4],joint_pos[5]]
exaxis_pos = [0,0,0,0]
error_joint = 0
count =1000
error = robot.ServoMoveStart()  #Servo motion start
print("Servo motion start",error)
while(count):

    error = robot.ServoJ(joint_pos,exaxis_pos)   #Joint space servo mode motion
    if error!=0:
        error_joint =error
    joint_pos[0] = joint_pos[0] - 0.01  #1 axis movement 0.1 degrees each time, movement 100 times
    exaxis_pos[0] = exaxis_pos[0]-0.01
    count = count - 1
    time.sleep(0.008)
print("The errcode of joint space servo mode motion ",error_joint)
error = robot.ServoMoveEnd()  #Servo motion end
print("Servo motion end",error)
# mode = 2
# n_pos = [0.0,0.0,0.5,0.0,0.0,0.0]   # Target Cartesian Position Increment
# error,desc_pos = robot.GetActualTCPPose()
# print("Current Cartesian position of robot ",desc_pos)
# count = 100
# error_cart =0
# error = robot.ServoMoveStart()  #Servo motion start
# print("The errcode of Servo motion start",error)
# while(count):
#     error = robot.ServoCart(mode, n_pos, vel=40)   # Cartesian space servo mode motion
#     if error!=0:
#         error_cart =error
#     count = count - 1
#     time.sleep(0.008)
# print("The errcode of cartesian space servo mode motion", error_cart)
# error = robot.ServoMoveEnd()  #Servo motion end
# print("The errcode of servo motion end",error)



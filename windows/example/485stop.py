from fairino import Robot
import random
import time

# 与机器人控制器建立连接，连接成功返回一个机器人对象
robot = Robot.RPC('192.168.58.2')

error = robot.AuxServoSetParam(1,1,1,1,130172,15.45)
print("AuxServoSetParam return",error)

error = robot.AuxServoEnable(1,0)
print("AuxServoEnable return",error)
time.sleep(1)

error = robot.AuxServoSetControlMode(1, 1)
print("AuxServoSetControlMode return",error)
time.sleep(1)
error = robot.AuxServoEnable(1, 1)
print("AuxServoEnable return",error)
time.sleep(1)
error = robot.AuxServoHoming(1, 1, 10, 10, 100)
print("AuxServoHoming return",error)
time.sleep(4)
error = robot.AuxServoSetEmergencyStopAcc(4000,5000)
print("AuxServoSetEmergencyStopAcc return",error)
time.sleep(1)
error = robot.AuxServoGetEmergencyStopAcc()
print("AuxServoGetEmergencyStopAcc return",error)
time.sleep(1)

error=robot.AuxServoSetAcc(490, 500)
print("AuxServoSetAcc",error)
error =robot.AuxServoSetTargetSpeed(1, 500, 100)
print("AuxServoSetTargetSpeed", error)

error,joint =robot.GetActualJointPosDegree()
print("GetActualJointPosDegree",error,joint)

joint[0]=joint[0]+20
error = robot.MoveJ(joint,3,0,blendT=1)
print("MoveJ",error)
joint[0]=joint[0]-90
error = robot.MoveJ(joint,3,0,blendT=1)
print("MoveJ",error)

n=0
while n<10:
    time.sleep(0.1)
    for i in range(1):
        print("auxState.servoVel:", robot.robot_state_pkg.auxState.servoVel)
        print("auxState.servoState:", robot.robot_state_pkg.auxState.servoState )
        print("auxState.servoState_emergency stop:", (robot.robot_state_pkg.auxState.servoState >> 7)& 0x01)
    if 1==(robot.robot_state_pkg.auxState.servoState >> 7)& 0x01:
        n=n+1

time.sleep(4)
error = robot.ResetAllError()
print("ResetAllError",error)
time.sleep(4)

n=0
while n<10:
    time.sleep(0.1)
    for i in range(1):
        print("auxState.servoVel:", robot.robot_state_pkg.auxState.servoVel)
        print("auxState.servoState:", robot.robot_state_pkg.auxState.servoState )
        print("auxState.servoState_emergency stop:", (robot.robot_state_pkg.auxState.servoState >> 7)& 0x01)
    if 1==(robot.robot_state_pkg.auxState.servoState >> 7)& 0x01:
        n=n+1

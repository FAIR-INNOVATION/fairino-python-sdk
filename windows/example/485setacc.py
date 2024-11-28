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
# error = robot.AuxServoSetControlMode(1, 0)
# print("AuxServoSetControlMode return",error)
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


n=1
while n<10000:
    for i in range(1):
        print("extAxisStatus.pos:", i, robot.robot_state_pkg.extAxisStatus[i].pos)
        print("extAxisStatus.vel:", i, robot.robot_state_pkg.extAxisStatus[i].vel)
        print("extAxisStatus.errorCode:", i, robot.robot_state_pkg.extAxisStatus[i].errorCode)

    # error=robot.AuxServoSetAcc(4900, 5000)
    # print("AuxServoSetAcc",error)
    # error =robot.AuxServoSetTargetPos(1, 2000, 500, 100)
    # print("AuxServoSetTargetPos", error)
    # time.sleep(6)
    # error =robot.AuxServoSetTargetPos(1, 0, 500, 100)
    # print("AuxServoSetTargetPos", error)
    # time.sleep(15)
    #
    # # error=robot.AuxServoSetAcc(490, 500)
    # # print("AuxServoSetAcc",error)
    # error =robot.AuxServoSetTargetPos(1, 2000, 500, 5)
    # print("AuxServoSetTargetPos", error)
    # time.sleep(7)
    # error =robot.AuxServoSetTargetPos(1, 0, 500, 5)
    # print("AuxServoSetTargetPos", error)
    # time.sleep(15)


    error=robot.AuxServoSetAcc(4900, 5000)
    print("AuxServoSetAcc",error)
    error =robot.AuxServoSetTargetSpeed(1, 500, 100)
    print("AuxServoSetTargetSpeed", error)
    time.sleep(3)
    error =robot.AuxServoSetTargetSpeed(1, -500, 100)
    print("AuxServoSetTargetSpeed", error)
    time.sleep(3)

    error =robot.AuxServoSetTargetSpeed(1, 500, 5)
    print("AuxServoSetTargetSpeed", error)
    time.sleep(3)
    error =robot.AuxServoSetTargetPos(1, 0, -500, 5)
    print("AuxServoSetTargetPos", error)
    time.sleep(3)

    # error=robot.AuxServoSetAcc(490, 500)
    # print("AuxServoSetAcc",error)
    # error =robot.AuxServoSetTargetPos(1, 2000, 500, 5)
    # print("AuxServoSetTargetPos", error)
    # time.sleep(7)
    # error =robot.AuxServoSetTargetPos(1, 0, 500, 5)
    # print("AuxServoSetTargetPos", error)
    # time.sleep(15)

    n=n+1
    time.sleep(0.5)
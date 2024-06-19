from time import sleep
from fairino import Robot
# A connection is established with the robot controller. A successful connection returns a robot object
ret = robot = Robot.RPC('192.168.58.2')
ret =robot.AuxServoEnable(1,0)#You must disable befor modify the control mode
print("AuxServoEnable(0)",ret)
sleep(3)

ret =robot.AuxServoSetControlMode(1,0)#Set the control mode to pos mode
print("AuxServoSetControlMode",ret)
sleep(3)

ret =robot.AuxServoEnable(1,1)#You must enable after set control mode
print("AuxServoEnable(1)",ret)
sleep(3)

ret =robot.AuxServoHoming(1,1,10,10)#Servo homing
print("AuxServoHoming",ret)
sleep(5)

ret =robot.AuxServoGetStatus(1)#Get Servo status
print("AuxServoGetStatus",ret)
sleep(1)
i=1
while(i<5):
    ret =robot.AuxServoSetTargetPos(1,300*i,30)#pos mode motion,speed 30
    print("AuxServoSetTargetPos",ret)
    sleep(11)
    ret =robot.AuxServoGetStatus(1)# Get Servo status
    print("AuxServoGetStatus",ret)
    sleep(1)
    i=i+1

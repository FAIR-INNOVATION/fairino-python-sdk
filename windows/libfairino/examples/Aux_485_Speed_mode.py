from time import sleep
from fairino import Robot
# A connection is established with the robot controller. A successful connection returns a robot object
ret = robot = Robot.RPC('192.168.58.2')
ret =robot.AuxServoEnable(1,0)#You must disable befor modify the control mode
print("AuxServoEnable(0)",ret)
sleep(3)

ret = robot.AuxServoSetControlMode(1, 1)  #Set the control mode to speed mode
print("AuxServoSetControlMode",ret)
sleep(3)

ret =robot.AuxServoEnable(1,1) #You must enable after set control mode
print("AuxServoEnable(1)",ret)
sleep(3)

ret =robot.AuxServoHoming(1,1,10,10) #Servo homing
print("AuxServoHoming",ret)
sleep(5)

ret =robot.AuxServoGetStatus(1)# Get 485 extended axis servo status
print("AuxServoGetStatus",ret)
sleep(1)

ret = robot.AuxServoSetTargetSpeed(1, 30)  # speed mode motion，speed 30
print("AuxServoSetTargetSpeed", ret)
sleep(10)

ret = robot.AuxServoGetStatus(1)  # Get 485 extended axis servo status
print("AuxServoGetStatus", ret)
sleep(1)

ret = robot.AuxServoSetTargetSpeed(1, 60)  # speed mode motion，speed 60
print("AuxServoSetTargetSpeed", ret)
sleep(10)
ret = robot.AuxServoGetStatus(1)  # Get 485 extended axis servo status
print("AuxServoGetStatus", ret)
sleep(1)

ret = robot.AuxServoSetTargetSpeed(1, 0)  # before end speed mode motion, you should sete speed to 0
print("AuxServoSetTargetSpeed", ret)
sleep(3)
ret = robot.AuxServoGetStatus(1)  # Get 485 extended axis servo status
print("AuxServoGetStatus", ret)
sleep(1)
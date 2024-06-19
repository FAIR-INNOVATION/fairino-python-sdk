from time import sleep
from fairino import Robot
# A connection is established with the robot controller. A successful connection returns a robot object
ret = robot = Robot.RPC('192.168.58.2')

ret =robot.AuxServoSetParam(1,1,1,1,131072,15.45)# Set 485 extended axis parameters
print("AuxServoSetParam",ret)
sleep(1)

ret =robot.AuxServoGetParam(1)# Get 485 extended axis configuration parameters
print("AuxServoGetParam",ret)
sleep(1)

ret =robot.AuxServoGetStatus(1)# Get 485 extended axis servo status
print("AuxServoGetStatus",ret)
sleep(1)

ret =robot.AuxServoClearError(1)# Clear 485 extended axis error message
print("AuxServoClearError",ret)
sleep(1)

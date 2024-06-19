from fairino import Robot
import time
 # A connection is established with the robot controller. A successful connection returns a robot object
robot = Robot.RPC('192.168.58.2')
#The UDP extension axis is disabled
error = robot.ExtAxisServoOn(1,0)
print("ExtAxisServoOn return:",error)
#The UDP extension axis is enabled
error = robot.ExtAxisServoOn(1,1)
print("ExtAxisServoOn return:",error)
#The UDP extension axis is homed
error = robot.ExtAxisSetHoming(1,0,40,40)
print("ExtAxisSetHoming return:",error)
time.sleep(1)
#The UDP extension axis starts Jog
error = robot.ExtAxisStartJog(1,1,20,20,20)
print("ExtAxisStartJog return:",error)
time.sleep(1)
#The UDP extension axis stops Jog
error = robot.ExtAxisStopJog(1)
print("ExtAxisStopJog return:",error)
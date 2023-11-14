from fairino import Robot
import time
# A connection is established with the robot controller. A successful connection returns a robot object
robot = Robot.RPC('192.168.58.2')
param=[1,10000,200,0,0,20]
ret = robot.ConveyorSetParam(param)
print("Set Conveyor Param",ret)

time.sleep(1)
cmp = [0.00, 0.00, 0.00]
ret1 = robot.ConveyorCatchPointComp(cmp)
print("Conveyor catch point compensation ",ret1)



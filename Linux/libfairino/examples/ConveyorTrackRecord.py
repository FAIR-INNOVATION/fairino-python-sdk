from fairino import Robot
# A connection is established with the robot controller. A successful connection returns a robot object
robot = Robot.RPC('192.168.58.2')
status = 1
robot.ConveyorStartEnd(status)
ret = robot.ConveyorPointIORecord()
print("Convey record the IO detection points ",ret)
ret = robot.ConveyorPointARecord()
print("Convey record A point ",ret)
ret = robot. ConveyorRefPointRecord()
print("Convey record reference point ",ret)
ret = robot.ConveyorPointBRecord()
print("Convey record B point ",ret)





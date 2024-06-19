from fairino import Robot
# A connection is established with the robot controller. A successful connection returns a robot object
robot = Robot.RPC('192.168.58.2')
# Conveyor track
while(1):
    robot.MoveL([-333.597, 60.354, 404.341, -179.143, -0.778, 91.275],0,0)
    error =robot.ConveyorIODetect(1000)
    print("Conveyor IO detection ",error)
    error =robot.ConveyorGetTrackData(1)
    print("Conveyor get track data ",error)
    error =robot.ConveyorTrackStart(1)
    print("Conveyor track start ",error)
    error =robot.ConveyorTrackMoveL("cvrCatchPoint",0,0,vel = 60.0)
    print("Conveyor track moveL ",error)
    error =robot.MoveGripper(1,55,20,20,30000,0)
    print("Gripper control",error)
    error =robot.ConveyorTrackMoveL("cvrRaisePoint",0,0,vel = 60.0)
    print("Conveyor track moveL ",error)
    error = robot.ConveyorTrackEnd()
    print("Conveyor track end ",error)
    error = robot.MoveL([-333.625, -229.039, 404.340, -179.141, -0.778, 91.276], 0, 0,vel =30)
    error = robot.MoveL([-333.564, 332.204, 342.217, -179.145, -0.780, 91.268], 0, 0,vel =30)
    error = robot.MoveGripper(1,100,10,21,30000,0)
    print("Gripper control ",error)




from fairino import Robot
import time
# A connection is established with the robot controller. A successful connection returns a robot object
import time
import threading

# 与机器人控制器建立连接，连接成功返回一个机器人对象
robot = Robot.RPC('192.168.58.2')

import threading

def Trigger(robot):
    i = int(input("请输入一个数字以触发 (please input a number to trigger): "))
    rtn = robot.ConveyorComDetectTrigger()
    print(f"ConveyorComDetectTrigger retval is: {rtn}")

def ConveyorTest(robot):
    retval = 0
    index = 1
    max_time = 30000
    block = 0
    retval = 0
    startdescPose = [139.176, 4.717, 9.088, -179.999, -0.004, -179.990]
    startjointPos = [-34.129, -88.062, 97.839, -99.780, -90.003, -34.140]

    homePose = [139.177, 4.717, 69.084, -180.000, -0.004, -179.989]
    homejointPos = [-34.129, -88.618, 84.039, -85.423, -90.003, -34.140]

    exaxisPos = [0, 0, 0, 0]
    offdese = [0, 0, 0, 0, 0, 0]

    # Move to home position
    retval = robot.MoveL(desc_pos=homePose, tool=1, user=1)
    print(f"MoveL to safety retval is: {retval}")

    # Start trigger thread
    textT = threading.Thread(target=Trigger, args=(robot,))
    textT.daemon = True
    textT.start()

    # Conveyor operations
    retval = robot.ConveyorComDetect(10000)
    print(f"ConveyorComDetect retval is: {retval}")

    retval = robot.ConveyorGetTrackData(2)
    print(f"ConveyorGetTrackData retval is: {retval}")

    retval = robot.ConveyorTrackStart(2)
    print(f"ConveyorTrackStart retval is: {retval}")

    # Movement commands
    robot.MoveL(desc_pos=startdescPose, tool=1, user=1)
    robot.MoveL(desc_pos=startdescPose, tool=1, user=1)

    # End conveyor tracking
    retval = robot.ConveyorTrackEnd()
    print(f"ConveyorTrackEnd retval is: {retval}")

    # Return to home position
    robot.MoveL(desc_pos=homePose, tool=1, user=1)

ConveyorTest(robot)
from time import sleep
from fairino import Robot
# A connection is established with the robot controller. A successful connection returns a robot object
# 与机器人控制器建立连接，连接成功返回一个机器人对象
robot = Robot.RPC('192.168.58.2')
import time
import time


def testLaserConfig(self):
    robot.LaserTrackingSensorConfig("192.168.58.20", 5020)
    robot.LaserTrackingSensorSamplePeriod(20)
    robot.LoadPosSensorDriver(101)
    robot.LaserTrackingLaserOnOff(0, 0)
    time.sleep(3)
    robot.LaserTrackingLaserOnOff(1, 0)
    # robot.CloseRPC()


def testGetLaserPoint(self):
    name = "test1"
    data = [0.0] * 20

    rtn,data = robot.GetRobotTeachingPoint(name)
    print(rtn)
    print(
        f"{data[0]},{data[1]},{data[2]},{data[3]},{data[4]},{data[5]},{data[6]},{data[7]},{data[8]},{data[9]},{data[10]},{data[11]}")

    startjointPos = [data[6], data[7], data[8], data[9], data[10], data[11]]
    startdescPose = [data[0], data[1], data[2], data[3], data[4], data[5]]
    exaxisPos = [0.0] * 4
    offdese = [0.0] * 6
    robot.MoveL(desc_pos=startdescPose,tool= 1,user= 0,vel= 100,acc= 100,ovl= 100,blendR= -1,exaxis_pos= exaxisPos,search= 0,offset_flag= 0, offset_pos= offdese,overSpeedStrategy= 1,speedPercent= 1)
    # robot.CloseRPC()


def testMoveToLaserRecordStart(self):
    # startjointPos = [56.205, -117.951, 141.872, -118.149, -94.217, -122.176]
    # startdescPose = [-97.552, -282.855, 26.675, 174.182, -1.338, -91.707]
    startjointPos = [72.99,-88.919,118.547,-121.577,-86.819,-22.073]
    startdescPose = [-21.007,-479.198,59.807,-176.997,2.215,-174.825]
    exaxisPos = [0.0] * 4
    offdese = [0.0] * 6
    robot.MoveL(desc_pos=startdescPose,tool= 1,user= 0,vel= 100,acc= 100,ovl= 100,blendR= -1,exaxis_pos= exaxisPos,search= 0,offset_flag= 0, offset_pos= offdese,overSpeedStrategy= 1,speedPercent= 1)

    robot.LaserSensorRecord1(2, 10)

    # endjointPos = [68.809, -87.100, 121.120, -127.233, -95.038, -109.555]
    # enddescPose = [-103.555, -464.234, 13.076, 174.179, -1.344, -91.709]
    endjointPos = [56.544,-81.138,108.869,-116.822,-87.375,-30.364]
    enddescPose = [-184.659,-491.569,60.888,-179.457,2.724,176.899]
    robot.MoveL(desc_pos=enddescPose,tool= 1,user= 0,vel= 50,acc= 100,ovl= 100,blendR= -1,exaxis_pos= exaxisPos,search= 0,offset_flag= 0, offset_pos= offdese,overSpeedStrategy= 1,speedPercent= 1)

    robot.LaserSensorRecord1(0, 10)
    robot.MoveToLaserRecordStart(1, 30)
    # robot.CloseRPC()


def testMoveToLaserRecordEnd(self):
    startjointPos = [72.99,-88.919,118.547,-121.577,-86.819,-22.073]
    startdescPose = [-21.007,-479.198,59.807,-176.997,2.215,-174.825]
    exaxisPos = [0.0] * 4
    offdese = [0.0] * 6
    robot.MoveL(desc_pos=startdescPose,tool= 1,user= 0,vel= 100,acc= 100,ovl= 100,blendR= -1,exaxis_pos= exaxisPos,search= 0,offset_flag= 0, offset_pos= offdese,overSpeedStrategy= 1,speedPercent= 1)

    robot.LaserSensorRecord1(2, 10)

    endjointPos = [56.544,-81.138,108.869,-116.822,-87.375,-30.364]
    enddescPose = [-184.659,-491.569,60.888,-179.457,2.724,176.899]
    robot.MoveL(desc_pos=enddescPose,tool= 1,user= 0,vel= 50,acc= 100,ovl= 100,blendR= -1,exaxis_pos= exaxisPos,search= 0,offset_flag= 0, offset_pos= offdese,overSpeedStrategy= 1,speedPercent= 1)

    robot.LaserSensorRecord1(0, 10)
    robot.MoveToLaserRecordEnd(1, 30)
    # robot.CloseRPC()


def testLasertrack_xyz(self):
    startjointPos = [72.99,-88.919,118.547,-121.577,-86.819,-22.073]
    startdescPose = [-21.007,-479.198,59.807,-176.997,2.215,-174.825]
    exaxisPos = [0.0] * 4
    offdese = [0.0] * 6
    directionPoint = [0.0] * 3
    robot.MoveL(desc_pos=startdescPose,tool= 1,user= 0,vel= 100,acc= 100,ovl= 100,blendR= -1,exaxis_pos= exaxisPos,search= 0,offset_flag= 0, offset_pos= offdese,overSpeedStrategy= 1,speedPercent= 1)

    robot.LaserTrackingSearchStart_xyz(1, 100, 300, 1000, 2)
    robot.LaserTrackingSearchStop()
    rtn, seamjointPos, seamdescPose, tool, user, startexaxisPos = robot.GetLaserSeamPos(0, offdese)
    print(
        f"{seamjointPos[0]},{seamjointPos[1]},{seamjointPos[2]},{seamjointPos[3]},{seamjointPos[4]},{seamjointPos[5]},{seamdescPose[0]},{seamdescPose[1]},{seamdescPose[2]},{seamdescPose[3]},{seamdescPose[4]},{seamdescPose[5]}")
    print(f"{tool},{user},{startexaxisPos[0]},{startexaxisPos[1]},{startexaxisPos[2]},{startexaxisPos[3]}")
    robot.MoveToLaserSeamPos(1, 30, 0, 0, 0, offdese)
    # robot.CloseRPC()


def testLasertrack_point(self):
    name = "test1"
    data = [0.0] * 20
    startjointPos = [72.99,-88.919,118.547,-121.577,-86.819,-22.073]
    startdescPose = [-21.007,-479.198,59.807,-176.997,2.215,-174.825]
    exaxisPos = [0.0] * 4
    offdese = [0.0] * 6
    directionPoint = [0.0] * 3

    robot.MoveL(desc_pos=startdescPose,tool= 1,user= 0,vel= 100,acc= 100,ovl= 100,blendR= -1,exaxis_pos= exaxisPos,search= 0,offset_flag= 0, offset_pos= offdese,overSpeedStrategy= 1,speedPercent= 1)

    rtn, data = robot.GetRobotTeachingPoint(name)
    print(
        f"{data[0]},{data[1]},{data[2]},{data[3]},{data[4]},{data[5]},{data[6]},{data[7]},{data[8]},{data[9]},{data[10]},{data[11]}")
    directionPoint[0] = data[0]
    directionPoint[1] = data[1]
    directionPoint[2] = data[2]
    print(f"{directionPoint[0]},{directionPoint[1]},{directionPoint[2]}")

    robot.LaserTrackingSearchStart_point(directionPoint, 100, 500, 1000, 2)
    robot.LaserTrackingSearchStop()
    robot.MoveToLaserSeamPos(1, 30, 0, 0, 0, offdese)
    # robot.CloseRPC()


def testLaserRecordAndReplay(self):
    robot.OpenLuaUpload("D://zUP/CtrlDev_laser_ruiniu-0117.lua")
    time.sleep(2)
    robot.SetCtrlOpenLUAName(0, "CtrlDev_laser_ruiniu-0117.lua")
    robot.UnloadCtrlOpenLUA(0)
    robot.LoadCtrlOpenLUA(0)
    time.sleep(8)
    i = 0

    while i<1:
        startjointPos = [72.99,-88.919,118.547,-121.577,-86.819,-22.073]
        startdescPose = [-21.007,-479.198,59.807,-176.997,2.215,-174.825]
        exaxisPos = [0.0] * 4
        offdese = [0.0] * 6
        robot.MoveL(desc_pos=startdescPose,tool= 1,user= 0,vel= 100,acc= 100,ovl= 100,blendR= -1,exaxis_pos= exaxisPos,search= 0,offset_flag= 0, offset_pos= offdese,overSpeedStrategy= 1,speedPercent= 1)

        robot.LaserSensorRecord1(2, 10)

        endjointPos = [56.544, -81.138, 108.869, -116.822, -87.375, -30.364]
        enddescPose = [-184.659, -491.569, 60.888, -179.457, 2.724, 176.899]
        robot.MoveL(desc_pos=enddescPose,tool= 1,user= 0,vel= 50,acc= 100,ovl= 100,blendR= -1,exaxis_pos= exaxisPos,search= 0,offset_flag= 0, offset_pos= offdese,overSpeedStrategy= 1,speedPercent= 1)

        robot.LaserSensorRecord1(0, 10)
        robot.MoveToLaserRecordStart(1, 30)
        robot.LaserSensorReplay(10, 100)
        robot.MoveLTR()
        robot.LaserSensorRecord1(0, 10)
        i = i+1

    # robot.CloseRPC()


def testLasertrack(self):
    robot.OpenLuaUpload("D://zUP/CtrlDev_laser_ruiniu-0117.lua")
    time.sleep(2)
    robot.SetCtrlOpenLUAName(0, "CtrlDev_laser_ruiniu-0117.lua")
    robot.UnloadCtrlOpenLUA(0)
    robot.LoadCtrlOpenLUA(0)
    time.sleep(8)

    time.sleep(8)
    i = 0

    while i < 1:
        startjointPos = [72.99,-88.919,118.547,-121.577,-86.819,-22.073]
        startdescPose = [-21.007,-479.198,59.807,-176.997,2.215,-174.825]
        exaxisPos = [0.0] * 4
        offdese = [0.0] * 6
        directionPoint = [0.0] * 3
        robot.MoveL(desc_pos=startdescPose,tool= 1,user= 0,vel= 100,acc= 100,ovl= 100,blendR= -1,exaxis_pos= exaxisPos,search= 0,offset_flag= 0, offset_pos= offdese,overSpeedStrategy= 1,speedPercent= 1)

        robot.LaserTrackingSearchStart_xyz(1, 100, 300, 1000, 2)
        robot.LaserTrackingSearchStop()
        robot.MoveToLaserSeamPos(1, 30, 0, 0, 0, offdese)

        robot.LaserTrackingTrackOnOff(1, 3)
        endjointPos = [56.544,-81.138,108.869,-116.822,-87.375,-30.364]
        enddescPose = [-184.659,-491.569,60.888,-179.457,2.724,176.899]
        robot.MoveL(desc_pos=enddescPose,tool= 1,user= 0,vel= 20,acc= 100,ovl= 100,blendR= -1,exaxis_pos= exaxisPos,search= 0,offset_flag= 0, offset_pos= offdese,overSpeedStrategy= 1,speedPercent= 1)

        robot.LaserTrackingTrackOnOff(0, 3)
        i = i + 1
        print(i)
    # robot.CloseRPC()

# testLaserConfig(robot)
# time.sleep(0.5)
# testGetLaserPoint(robot)
# time.sleep(0.5)
# testMoveToLaserRecordStart(robot)
# time.sleep(0.5)
# testMoveToLaserRecordEnd(robot)
# time.sleep(0.5)
# testLasertrack_xyz(robot)
# time.sleep(0.5)
# testLasertrack_point(robot)
# time.sleep(0.5)
# testLaserRecordAndReplay(robot)
# time.sleep(0.5)
# testLasertrack(robot)
# time.sleep(0.5)
# robot.CloseRPC()


rtn = robot.ResetAllError()
print(rtn)
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
    robot.CloseRPC()


def testGetLaserPoint(self):
    name = "laserPoint"
    data = [0.0] * 20

    rtn,data = robot.GetRobotTeachingPoint(name)
    print(
        f"{data[0]},{data[1]},{data[2]},{data[3]},{data[4]},{data[5]},{data[6]},{data[7]},{data[8]},{data[9]},{data[10]},{data[11]}")

    startjointPos = [data[6], data[7], data[8], data[9], data[10], data[11]]
    startdescPose = [data[0], data[1], data[2], data[3], data[4], data[5]]
    exaxisPos = [0.0] * 4
    offdese = [0.0] * 6
    robot.MoveL(desc_pos=startdescPose,tool= 1,user= 0,vel= 100,acc= 100,ovl= 100,blendR= -1,exaxis_pos= exaxisPos,search= 0,offset_flag= 0, offset_pos= offdese,overSpeedStrategy= 1,speedPercent= 1)
    robot.CloseRPC()


def testMoveToLaserRecordStart(self):
    startjointPos = [56.205, -117.951, 141.872, -118.149, -94.217, -122.176]
    startdescPose = [-97.552, -282.855, 26.675, 174.182, -1.338, -91.707]
    exaxisPos = [0.0] * 4
    offdese = [0.0] * 6
    robot.MoveL(desc_pos=startdescPose,tool= 1,user= 0,vel= 100,acc= 100,ovl= 100,blendR= -1,exaxis_pos= exaxisPos,search= 0,offset_flag= 0, offset_pos= offdese,overSpeedStrategy= 1,speedPercent= 1)

    robot.LaserSensorRecord1(2, 10)

    endjointPos = [68.809, -87.100, 121.120, -127.233, -95.038, -109.555]
    enddescPose = [-103.555, -464.234, 13.076, 174.179, -1.344, -91.709]
    robot.MoveL(desc_pos=enddescPose,tool= 1,user= 0,vel= 50,acc= 100,ovl= 100,blendR= -1,exaxis_pos= exaxisPos,search= 0,offset_flag= 0, offset_pos= offdese,overSpeedStrategy= 1,speedPercent= 1)

    robot.LaserSensorRecord1(0, 10)
    robot.MoveToLaserRecordStart(1, 30)
    robot.CloseRPC()


def testMoveToLaserRecordEnd(self):
    startjointPos = [56.205, -117.951, 141.872, -118.149, -94.217, -122.176]
    startdescPose = [-97.552, -282.855, 26.675, 174.182, -1.338, -91.707]
    exaxisPos = [0.0] * 4
    offdese = [0.0] * 6
    robot.MoveL(desc_pos=startdescPose,tool= 1,user= 0,vel= 100,acc= 100,ovl= 100,blendR= -1,exaxis_pos= exaxisPos,search= 0,offset_flag= 0, offset_pos= offdese,overSpeedStrategy= 1,speedPercent= 1)

    robot.LaserSensorRecord1(2, 10)

    endjointPos = [68.809, -87.100, 121.120, -127.233, -95.038, -109.555]
    enddescPose = [-103.555, -464.234, 13.076, 174.179, -1.344, -91.709]
    robot.MoveL(desc_pos=enddescPose,tool= 1,user= 0,vel= 50,acc= 100,ovl= 100,blendR= -1,exaxis_pos= exaxisPos,search= 0,offset_flag= 0, offset_pos= offdese,overSpeedStrategy= 1,speedPercent= 1)

    robot.LaserSensorRecord1(0, 10)
    robot.MoveToLaserRecordEnd(1, 30)
    robot.CloseRPC()


def testLasertrack_xyz(self):
    startjointPos = [56.205, -117.951, 141.872, -118.149, -94.217, -122.176]
    startdescPose = [-97.552, -282.855, 26.675, 174.182, -1.338, -91.707]
    exaxisPos = [0.0] * 4
    offdese = [0.0] * 6
    directionPoint = [0.0] * 3
    robot.MoveL(desc_pos=startdescPose,tool= 1,user= 0,vel= 100,acc= 100,ovl= 100,blendR= -1,exaxis_pos= exaxisPos,search= 0,offset_flag= 0, offset_pos= offdese,overSpeedStrategy= 1,speedPercent= 1)

    robot.LaserTrackingSearchStart_xyz(3, 100, 300, 1000, 3)
    robot.LaserTrackingSearchStop()
    robot.MoveToLaserSeamPos(1, 30, 0, 0, 0, offdese)
    robot.CloseRPC()


def testLasertrack_point(self):
    name = "laserEnd"
    data = [0.0] * 20
    startjointPos = [56.205, -117.951, 141.872, -118.149, -94.217, -122.176]
    startdescPose = [-97.552, -282.855, 26.675, 174.182, -1.338, -91.707]
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

    robot.LaserTrackingSearchStart_point(directionPoint, 100, 500, 1000, 3)
    robot.LaserTrackingSearchStop()
    robot.MoveToLaserSeamPos(1, 30, 0, 0, 0, offdese)
    robot.CloseRPC()


def testLaserRecordAndReplay(self):
    robot.OpenLuaUpload("D://zUP/CtrlDev_laser_ruiniu-0117.lua")
    time.sleep(2)
    robot.SetCtrlOpenLUAName(0, "CtrlDev_laser_ruiniu-0117.lua")
    robot.UnloadCtrlOpenLUA(0)
    robot.LoadCtrlOpenLUA(0)
    time.sleep(8)
    i = 0

    while i<10:
        startjointPos = [56.205, -117.951, 141.872, -118.149, -94.217, -122.176]
        startdescPose = [-97.552, -282.855, 26.675, 174.182, -1.338, -91.707]
        exaxisPos = [0.0] * 4
        offdese = [0.0] * 6
        robot.MoveL(desc_pos=startdescPose,tool= 1,user= 0,vel= 100,acc= 100,ovl= 100,blendR= -1,exaxis_pos= exaxisPos,search= 0,offset_flag= 0, offset_pos= offdese,overSpeedStrategy= 1,speedPercent= 1)

        robot.LaserSensorRecord1(2, 10)

        endjointPos = [68.809, -87.100, 121.120, -127.233, -95.038, -109.555]
        enddescPose = [-103.555, -464.234, 13.076, 174.179, -1.344, -91.709]
        robot.MoveL(desc_pos=enddescPose,tool= 1,user= 0,vel= 50,acc= 100,ovl= 100,blendR= -1,exaxis_pos= exaxisPos,search= 0,offset_flag= 0, offset_pos= offdese,overSpeedStrategy= 1,speedPercent= 1)

        robot.LaserSensorRecord1(0, 10)
        robot.MoveToLaserRecordStart(1, 30)
        robot.LaserSensorReplay(10, 100)
        robot.MoveLTR()
        robot.LaserSensorRecord1(0, 10)
        i = i+1

    robot.CloseRPC()


def testLasertrack(self):
    robot.OpenLuaUpload("D://zUP/CtrlDev_laser_ruiniu-0117.lua")
    time.sleep(2)
    robot.SetCtrlOpenLUAName(0, "CtrlDev_laser_ruiniu-0117.lua")
    robot.UnloadCtrlOpenLUA(0)
    robot.LoadCtrlOpenLUA(0)
    time.sleep(8)

    time.sleep(8)
    i = 0

    while i < 10:
        startjointPos = [56.205, -117.951, 141.872, -118.149, -94.217, -122.176]
        startdescPose = [-97.552, -282.855, 26.675, 174.182, -1.338, -91.707]
        exaxisPos = [0.0] * 4
        offdese = [0.0] * 6
        directionPoint = [0.0] * 3
        robot.MoveL(desc_pos=startdescPose,tool= 1,user= 0,vel= 100,acc= 100,ovl= 100,blendR= -1,exaxis_pos= exaxisPos,search= 0,offset_flag= 0, offset_pos= offdese,overSpeedStrategy= 1,speedPercent= 1)

        robot.LaserTrackingSearchStart_xyz(3, 100, 300, 1000, 3)
        robot.LaserTrackingSearchStop()
        robot.MoveToLaserSeamPos(1, 30, 0, 0, 0, offdese)

        robot.LaserTrackingTrackOnOff(1, 3)
        endjointPos = [68.809, -87.100, 121.120, -127.233, -95.038, -109.555]
        enddescPose = [-103.555, -464.234, 13.076, 174.179, -1.344, -91.709]
        robot.MoveL(desc_pos=enddescPose,tool= 1,user= 0,vel= 20,acc= 100,ovl= 100,blendR= -1,exaxis_pos= exaxisPos,search= 0,offset_flag= 0, offset_pos= offdese,overSpeedStrategy= 1,speedPercent= 1)

        robot.LaserTrackingTrackOnOff(0, 3)
        i = i + 1
        print(i)
    robot.CloseRPC()

# testLaserConfig(robot)
# testGetLaserPoint(robot)
# testMoveToLaserRecordStart(robot)
# testMoveToLaserRecordEnd(robot)
# testLasertrack_xyz(robot)
# testLasertrack_point(robot)
# testLaserRecordAndReplay(robot)
# testLasertrack(robot)
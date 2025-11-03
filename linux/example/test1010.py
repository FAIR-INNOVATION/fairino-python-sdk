from time import sleep
from fairino import Robot
from ctypes import sizeof
# A connection is established with the robot controller. A successful connection returns a robot object
# 与机器人控制器建立连接，连接成功返回一个机器人对象
robot = Robot.RPC('192.168.58.2')
import time

import time

def testLasertrackMoveC(self):
    #上传并加载开放协议文件
    #robot.OpenLuaUpload("E://openlua/CtrlDev_laser_ruiniu-0117.lua")
    #time.sleep(2)
    #robot.SetCtrlOpenLUAName(0, "CtrlDev_laser_ruiniu-0117.lua")
    #robot.UnloadCtrlOpenLUA(0)
    #robot.LoadCtrlOpenLUA(0)
    #time.sleep(8)
    robot.ResetAllError()
    cnt = 1
    while cnt < 2:
        #运动到需要寻位的起始点
        startjointPos = [40.947, -133.649, 128.497, -108.428, -87.159, -21.741]
        startdescPose = [-167.396, -301.742, 224.468, -157.008, -6.084, 152.043]
        exaxisPos = [0.0] * 4
        offdese = [0.0] * 6
        directionPoint = [0.0] * 3
        rtn = robot.MoveL(desc_pos=startdescPose,tool= 1,user= 0,vel= 50,acc= 100,ovl= 100,blendR= -1,exaxis_pos= exaxisPos,search= 0,offset_flag= 0,offset_pos= offdese,velAccParamMode= 1,overSpeedStrategy= 1)
        print(rtn)
        time.sleep(2)
        #沿着-y方向开始寻位
        ret = robot.LaserTrackingSearchStart_xyz(0, 100, 300, 1000, 2)
        robot.LaserTrackingSearchStop()
        #如果寻位成功
        if ret == 0:
            #运动到寻位点
            robot.MoveToLaserSeamPos(1, 30, 0, 0, 0, offdese)
            #开始沿着寻位点进行激光跟踪
            robot.LaserTrackingTrackOnOff(1, 2)
            midjointPos = [23.925, -68.391, 72.649, -89.970, -102.641, 102.979]
            middescPose = [-561.957, -282.616, 179.994, -166.732, -1.366, 11.262]
            endjointPos = [-11.146, -110.681, 112.893, -71.999, -83.325, 208.723]
            enddescPose = [-250.875, -93.272, 182.343, -159.126, 4.036, -130.316]
            # robot.MoveC(desc_pos_p= middescPose,tool_p= 1,user_p= 0,vel_p= 30,acc_p= 100,exaxis_pos_p= exaxisPos,offset_flag_p= 0,offset_pos_p= offdese,desc_pos_t= enddescPose,tool_t= 1,user_t= 0,vel_t= 30,acc_t= 100,exaxis_pos_t= exaxisPos,offset_flag_t= 0,offset_pos_t= offdese,ovl= 100, blendR= -1,velAccParamMode= 0)
            robot.Circle(desc_pos_p=middescPose,tool_p= 1,user_p= 0,vel_p= 30,acc_p= 100,exaxis_pos_p= exaxisPos,desc_pos_t= enddescPose,tool_t= 1,user_t= 0,vel_t= 30,acc_t= 100,exaxis_pos_t= exaxisPos,ovl= 100,offset_flag= 0, offset_pos= offdese,oacc= 100, blendR= -1,velAccParamMode= 0)

            #停止跟踪
            robot.LaserTrackingTrackOnOff(0, 2)

        cnt += 1
    robot.CloseRPC()

testLasertrackMoveC(robot)
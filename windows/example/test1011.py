from time import sleep
from fairino import Robot
from ctypes import sizeof
# A connection is established with the robot controller. A successful connection returns a robot object
# 与机器人控制器建立连接，连接成功返回一个机器人对象
robot = Robot.RPC('192.168.57.2')
import time


def TestSensitivityCalib(self):
    rtn = robot.JointSensitivityEnable(1)
    print(f"JointSensitivityEnable rtn is {rtn}")
    curJPos = [0.0] * 6
    rtn, curJPos = robot.GetActualJointPosDegree(0)

    jointPos1 = [curJPos[0], 0.0, 0.0, -90.0, 0.02, curJPos[5]]
    descPos1 = [0.0] * 6
    rtn , descPos1 = robot.GetForwardKin(jointPos1)

    epos = [0.0] * 4
    offset_pos = [0.0] * 6

    robot.MoveJ(joint_pos=jointPos1, desc_pos=descPos1,tool= 0,user= 0,vel= 100,acc= 100,ovl= 100,exaxis_pos= epos,blendT= -1,offset_flag= 0,offset_pos= offset_pos)

    time.sleep(0.2)
    rtn = robot.JointSensitivityCollect()
    print(f"JointSensitivityCollect 1 rtn is {rtn}")
    time.sleep(0.1)
    jointPos2 = [curJPos[0], -30.0, 0.0, -90.0, 0.02, curJPos[5]]
    descPos2 = [0.0] * 6
    rtn, descPos2 = robot.GetForwardKin(jointPos2)

    robot.MoveJ(joint_pos=jointPos2, desc_pos=descPos2,tool= 0,user= 0,vel= 100,acc= 100,ovl= 100,exaxis_pos= epos,blendT= -1,offset_flag= 0,offset_pos= offset_pos)

    time.sleep(0.1)
    rtn = robot.JointSensitivityCollect()
    print(f"JointSensitivityCollect 2 rtn is {rtn}")
    time.sleep(0.1)

    jointPos3 = [curJPos[0], -60.0, 0.0, -90.0, 0.02, curJPos[5]]
    descPos3 = [0.0] * 6
    rtn, descPos3 = robot.GetForwardKin(jointPos3)
    robot.MoveJ(joint_pos=jointPos3,desc_pos= descPos3,tool= 0,user= 0,vel= 100,acc= 100,ovl= 100,exaxis_pos= epos,blendT= -1,offset_flag= 0,offset_pos= offset_pos)

    time.sleep(0.1)
    rtn = robot.JointSensitivityCollect()
    print(f"JointSensitivityCollect 3 rtn is {rtn}")
    time.sleep(0.1)

    jointPos4 = [curJPos[0], -90.0, 0.0, -90.0, 0.02, curJPos[5]]
    descPos4 = [0.0] * 6
    rtn, descPos4 = robot.GetForwardKin(jointPos4)
    robot.MoveJ(joint_pos=jointPos4, desc_pos= descPos4,tool= 0,user= 0,vel= 100,acc= 100,ovl= 100,exaxis_pos= epos,blendT= -1,offset_flag= 0,offset_pos= offset_pos)

    time.sleep(0.1)
    rtn = robot.JointSensitivityCollect()
    print(f"JointSensitivityCollect 4 rtn is {rtn}")
    time.sleep(0.1)

    jointPos5 = [curJPos[0], -120.0, 0.0, -90.0, 0.02, curJPos[5]]
    descPos5 = [0.0] * 6
    rtn, descPos5 = robot.GetForwardKin(jointPos5)
    robot.MoveJ(joint_pos=jointPos5, desc_pos= descPos5,tool= 0,user= 0,vel= 100,acc= 100,ovl= 100,exaxis_pos= epos,blendT= -1,offset_flag= 0,offset_pos= offset_pos)

    time.sleep(0.1)
    rtn = robot.JointSensitivityCollect()
    print(f"JointSensitivityCollect 5 rtn is {rtn}")
    time.sleep(0.1)

    jointPos6 = [curJPos[0], -150.0, 0.0, -90.0, 0.02, curJPos[5]]
    descPos6 = [0.0] * 6
    rtn, descPos6 = robot.GetForwardKin(jointPos6)
    robot.MoveJ(joint_pos=jointPos6, desc_pos= descPos6,tool= 0,user= 0,vel= 100,acc= 100,ovl= 100,exaxis_pos= epos,blendT= -1,offset_flag= 0,offset_pos= offset_pos)

    time.sleep(0.1)
    rtn = robot.JointSensitivityCollect()
    print(f"JointSensitivityCollect 6 rtn is {rtn}")
    time.sleep(0.1)

    jointPos7 = [curJPos[0], -180.0, 0.0, -90.0, 0.02, curJPos[5]]
    descPos7 = [0.0] * 6
    rtn, descPos7 = robot.GetForwardKin(jointPos7)
    robot.MoveJ(joint_pos=jointPos7, desc_pos= descPos7,tool= 0,user= 0,vel= 100,acc= 100,ovl= 100,exaxis_pos= epos,blendT= -1,offset_flag= 0,offset_pos= offset_pos)

    time.sleep(0.1)
    rtn = robot.JointSensitivityCollect()
    print(f"JointSensitivityCollect 7 rtn is {rtn}")
    time.sleep(0.1)

    calibResult = [0.0] * 6
    rtn,calibResult = robot.JointSensitivityCalibration()
    print(f"JointSensitivityCalibration rtn is {rtn}")
    rtn = robot.JointSensitivityEnable(0)
    print(f"JointSensitivityEnable rtn is {rtn}")

    print(f"jointSensor Calib result is {calibResult[0]},{calibResult[1]},{calibResult[2]},{calibResult[3]},{calibResult[4]},{calibResult[5]}")

    robot.CloseRPC()

TestSensitivityCalib(robot)
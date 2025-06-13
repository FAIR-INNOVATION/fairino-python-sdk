from fairino import Robot
import time
# A connection is established with the robot controller. A successful connection returns a robot object
import time
import threading

# 与机器人控制器建立连接，连接成功返回一个机器人对象
robot = Robot.RPC('192.168.58.2')

def TestBlend(robot):
    middescPoseCir1 = [-435.414, -342.926, 309.205, -171.382, -4.513, 171.520]
    midjointPosCir1 = [26.804, -79.866, 106.642, -125.433, -85.562, -54.721]
    enddescPoseCir1 = [-524.862, -217.402, 308.459, -171.425, -4.810, 156.088]
    endjointPosCir1 = [11.399, -78.055, 104.603, -125.421, -85.770, -54.721]

    middescPoseCir2 = [-482.691, -587.899, 318.594, -171.001, -4.999, -172.996]
    midjointPosCir2 = [42.314, -53.600, 67.296, -112.969, -85.533, -54.721]
    enddescPoseCir2 = [-403.942, -489.061, 317.038, -163.189, -10.425, -175.627]
    endjointPosCir2 = [39.959, -70.616, 96.679, -134.243, -82.276, -54.721]

    middescPoseMoveC = [-435.414, -342.926, 309.205, -171.382, -4.513, 171.520]
    midjointPosMoveC = [26.804, -79.866, 106.642, -125.433, -85.562, -54.721]
    enddescPoseMoveC = [-524.862, -217.402, 308.459, -171.425, -4.810, 156.088]
    endjointPosmoveC = [11.399, -78.055, 104.603, -125.421, -85.770, -54.721]

    middescPoseCir3 = [-435.414, -342.926, 309.205, -171.382, -4.513, 171.520]
    midjointPosCir3 = [26.804, -79.866, 106.642, -125.433, -85.562, -54.721]
    enddescPoseCir3 = [-569.505, -405.378, 357.596, -172.862, -10.939, 171.108]
    endjointPosCir3 = [27.138, -63.750, 78.586, -117.861, -90.588, -54.721]

    middescPoseCir4 = [-482.691, -587.899, 318.594, -171.001, -4.999, -172.996]
    midjointPosCir4 = [42.314, -53.600, 67.296, -112.969, -85.533, -54.721]
    enddescPoseCir4 = [-569.505, -405.378, 357.596, -172.862, -10.939, 171.108]
    endjointPosCir4 = [27.138, -63.750, 78.586, -117.861, -90.588, -54.721]

    startdescPose = [-569.505, -405.378, 357.596, -172.862, -10.939, 171.108]
    startjointPos = [27.138, -63.750, 78.586, -117.861, -90.588, -54.721]

    linedescPose = [-403.942, -489.061, 317.038, -163.189, -10.425, -175.627]
    linejointPos = [39.959, -70.616, 96.679, -134.243, -82.276, -54.721]

    exaxisPos = [0, 0, 0, 0]
    offdese = [0, 0, 0, 0, 0, 0]

    robot.MoveJ(joint_pos=startjointPos, tool=3, user=0, vel=100)
    robot.Circle(desc_pos_p=middescPoseCir1, tool_p=3, user_p=0, vel_p=100, desc_pos_t=enddescPoseCir1, tool_t=3,user_t=0, vel_t=100, offset_flag=-1,oacc=100, blendR=20)
    robot.Circle(desc_pos_p=middescPoseCir2, tool_p=3, user_p=0, vel_p=100, desc_pos_t=enddescPoseCir2, tool_t=3,user_t=0, vel_t=100, offset_flag=-1,oacc=100, blendR=20)
    robot.MoveC(desc_pos_p=middescPoseMoveC, tool_p=3, user_p=0, vel_p=100,desc_pos_t=enddescPoseMoveC,tool_t=3,user_t=0,vel_t=100, blendR=20)
    robot.Circle(desc_pos_p=middescPoseCir3, tool_p=3, user_p=0, vel_p=100, desc_pos_t=enddescPoseCir3, tool_t=3,user_t=0, vel_t=100, offset_flag=-1,oacc=100, blendR=20)
    robot.MoveL(desc_pos=linedescPose, tool=3, user=0, vel=100,blendMode=0)
    robot.Circle(desc_pos_p=middescPoseCir4, tool_p=3, user_p=0, vel_p=100, desc_pos_t=enddescPoseCir4, tool_t=3,user_t=0, vel_t=100, offset_flag=-1,oacc=100, blendR=20)


TestBlend(robot)
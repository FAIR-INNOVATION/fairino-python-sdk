from fairino import Robot
import time
# A connection is established with the robot controller. A successful connection returns a robot object
import time
import threading

# 与机器人控制器建立连接，连接成功返回一个机器人对象
robot = Robot.RPC('192.168.58.2')

def TestInverseKen(self):
    dcs1 = [32.316, -232.029, 1063.415, 90.159, 18.376, 36.575]
    dcs2 = [105.25, -170.914, 1076.283, 87.032, 25.94, 54.644]
    dcs3 = [79.164, 81.645, 1045.609, 133.691, -73.265, 162.726]
    dcs4 = [298.779, -104.112, 298.242, 179.631, -0.628, -166.481]

    # error, inverseRtn = robot.GetInverseKin(type=0,desc_pos=dcs1,config=-1)
    # print("dcs1 getinverse rtn is ",inverseRtn)
    # error, inverseRtn = robot.GetInverseKin(type=0, desc_pos=dcs2, config=-1)
    # print("dcs2 getinverse rtn is ",inverseRtn)

    # error, inverseRtn = robot.GetInverseKin(type=0, desc_pos=dcs3, config=-1)
    # print("dcs3 getinverse rtn is ",inverseRtn)
    # error, inverseRtn = robot.GetInverseKin(type=0, desc_pos=dcs4, config=-1)
    # print("dcs4 getinverse rtn is ",inverseRtn)

    jpos1 = [56.999, -59.002, 56.996, -96.552, 60.392, -90.005]
    error,forwordResult = robot.GetForwardKin(joint_pos=jpos1)
    print("jpos1 forwordResult rtn is ",forwordResult)

TestInverseKen(robot)
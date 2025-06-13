from fairino import Robot
import time
# A connection is established with the robot controller. A successful connection returns a robot object
import time
import threading

# 与机器人控制器建立连接，连接成功返回一个机器人对象
robot = Robot.RPC('192.168.58.2')


def test(robot):
    error,state = robot.ExtAxisGetCoord()
    print("扩展轴坐标系为：",state[0],",",state[1],",",state[2],",",state[3],",",state[3],",",state[4],",",state[5])

def TestBlend(robot):
    DP1 = [-324.688, -512.411, 319.936, 177.834, -13.926, -123.378]
    JP1 = [47.944, -74.115, 99.306, -129.280, -90.062, -98.421]

    DP2 = [-388.074, -328.779, 340.076, -159.121, 16.169, -174.291]
    JP2 = [23.798, -86.390, 105.682, -100.633, -65.192, -70.820]

    DP3 = [-492.692, -49.563, 375.256, 161.781, -14.476, 159.830]
    JP3 = [-1.812, -89.883, 108.067, -116.040, -111.809, -70.825]

    DP4 = [-432.689, -287.194, 305.739, -177.999, 1.920, -177.450]
    JP4 = [21.721, -83.395, 108.235, -113.684, -87.480, -70.821]

    DP5 = [-232.690, -287.193, 305.746, -177.999, 1.919, -177.450]
    JP5 = [34.158, -105.217, 128.305, -112.503, -87.290, -58.372]

    DP6 = [-232.695, -487.192, 305.744, -177.999, 1.919, -177.452]
    JP6 = [53.031, -80.893, 105.748, -115.179, -87.247, -39.476]

    JP7 = [38.933, -66.532, 86.532, -109.644, -87.251, -53.590]
    DP7 = [-432.695, -487.196, 305.749, -177.999, 1.918, -177.452]

    JP8 = [42.245, -82.011, 99.838, -116.087, -69.438, -70.824]
    DP8 = [-315.138, -471.802, 373.506, -157.941, -1.233, -155.671]

    DP9 = [-513.450, -302.627, 402.163, 171.249, -16.204, -176.411]
    JP9 = [22.919, -78.425, 92.035, -116.080, -103.583, -70.913]

    DP10 = [-428.141, -188.113, 351.314, 176.576, -19.670, 142.831]
    JP10 = [14.849, -92.942, 114.901, -121.601, -107.553, -38.881]

    DP11 = [-587.412, -70.091, 370.337, 177.676, -23.575, 127.293]
    JP11 = [0.209, -77.444, 96.217, -121.606, -110.075, -38.879]

    JP12 = [-21.947, -88.425, 108.395, -111.062, -77.881, -38.879]
    DP12 = [-498.493, 67.966, 345.644, -171.472, 8.710, 107.699]

    exaxisPos = [0, 0, 0, 0]
    offdese = [0, 0, 0, 0, 0, 0]

    robot.MoveJ(joint_pos=JP1,tool=0,user=0,vel=100)
    robot.MoveJ(joint_pos=JP2,tool=0,user=0,vel=100,blendT=200)
    robot.MoveJ(joint_pos=JP3,tool=0,user=0,vel=100,blendT=200)
    robot.MoveJ(joint_pos=JP4,tool=0,user=0,vel=100,blendT=200)

    robot.MoveL(desc_pos=DP5,tool=0,user=0,vel=100,blendMode=0,blendR=20)
    robot.MoveL(desc_pos=DP6,tool=0,user=0,vel=100,blendMode=1,blendR=20)
    robot.MoveL(desc_pos=DP7,tool=0,user=0,vel=100,blendMode=0,blendR=20)

    robot.MoveJ(joint_pos=JP8,tool=0,user=0,vel=100)
    error = robot.MoveC(desc_pos_p=DP9,tool_p=0,user_p=0,desc_pos_t=DP10,tool_t=0,user_t=0,vel_p=100,vel_t=100,blendR=30)
    error = robot.MoveC(desc_pos_p=DP11,tool_p=0,user_p=0,desc_pos_t=DP12,tool_t=0,user_t=0,vel_p=100,vel_t=100)



# test(robot)
TestBlend(robot)
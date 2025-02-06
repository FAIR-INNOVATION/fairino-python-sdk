from Cython.Utils import raise_error_if_module_name_forbidden

from fairino import Robot
import time
from datetime import datetime
# A connection is established with the robot controller. A successful connection returns a robot object
# robot = Robot.RPC('192.168.58.2')
# robot.CloseRPC()
# time.sleep(0.5)
# robot = None
# print("main_code:",robot.robot_state_pkg.main_code)
# robot1 = Robot.RPC('192.168.58.2')
# print("main_code:",robot1.robot_state_pkg.main_code)
# # robot1.CloseRPC()
# robot2 = Robot.RPC('192.168.58.2')
# print("main_code:",robot2.robot_state_pkg.main_code)


def movej_test(self):
    """机器人MoveJ测试"""
    JP = [137.678,-97.426,-59.893,-102.467,90.287,15.775]
    DP = [-329.379,437.127,646.486,-170.238,3.036,32.188]

    JP1 = [102.325,-95.509,-59.849,-106.768,90.294,15.754]
    DP1 = [-10.827,524.713,655.736,-172.496,2.409,-3.249]

    error = robot.MoveJ(joint_pos=JP,tool=0,user=0, vel=30)
    print("MoveJ return ",error)


    print("点1-jt_cur_pos0-应为:137.678-反馈为:", robot.robot_state_pkg.jt_cur_pos[0])
    print("点1-jt_cur_pos1-应为:-97.426-反馈为:", robot.robot_state_pkg.jt_cur_pos[1])
    print("点1-jt_cur_pos2-应为:-59.893-反馈为:", robot.robot_state_pkg.jt_cur_pos[2])
    print("点1-jt_cur_pos3-应为:-102.467-反馈为:", robot.robot_state_pkg.jt_cur_pos[3])
    print("点1-jt_cur_pos4-应为:90.287-反馈为:", robot.robot_state_pkg.jt_cur_pos[4])
    print("点1-jt_cur_pos5-应为:15.775-反馈为:", robot.robot_state_pkg.jt_cur_pos[5])

    error = robot.MoveJ(joint_pos=JP1, tool=0, user=0, vel=30)
    print("MoveJ return ", error)

    print("点2-jt_cur_pos0-应为:102.325-反馈为:", robot.robot_state_pkg.jt_cur_pos[0])
    print("点2-jt_cur_pos1-应为:-95.509-反馈为:", robot.robot_state_pkg.jt_cur_pos[1])
    print("点2-jt_cur_pos2-应为:-59.849-反馈为:", robot.robot_state_pkg.jt_cur_pos[2])
    print("点2-jt_cur_pos3-应为:-106.768-反馈为:", robot.robot_state_pkg.jt_cur_pos[3])
    print("点2-jt_cur_pos4-应为:90.294-反馈为:", robot.robot_state_pkg.jt_cur_pos[4])
    print("点2-jt_cur_pos5-应为:15.754-反馈为:", robot.robot_state_pkg.jt_cur_pos[5])

def test(self):
    JP = [137.678, -97.426, -59.893, -102.467, 90.287, 15.775]
    robot.MoveJ(joint_pos=JP, tool=20, user=0, vel=30)

    error = robot.GetRobotErrorCode()
    print("GetRobotErrorCode:", error)
    timestamp_ms = int(datetime.now().timestamp() * 1000)
    print("当前时间戳（毫秒级）:", timestamp_ms)

    robot.ResetAllError()

    error = robot.GetRobotErrorCode()
    print("GetRobotErrorCode:", error)
    timestamp_ms = int(datetime.now().timestamp() * 1000)
    print("当前时间戳（毫秒级）:", timestamp_ms)

    error = robot.GetRobotErrorCode()
    print("GetRobotErrorCode1:", error)

    time.sleep(0.00001)
    error = robot.GetRobotErrorCode()
    print("GetRobotErrorCode2:", error)

# def test0116(self):
#     robot = Robot.RPC('192.168.58.2')

# JP = [-17.099,-102.71,-114.101,-8.262,120.908,-18.8]
#
# JP1 = [-3.349, -102.695 ,-113.797,-7.265,121.789,-18.799]

JP = [75.019,-84.624,111.369,-118.036,-95.126,98.303]
DP = [-38.513,-503.946,300.078,-175.114,2.014,66.745]

JP1 = [101.209,-84.627,111.374,-118.037,-95.195,98.303]
DP1 = [187.759,-469.194,300.098,-175.045,2.026,92.939]

i = 1
# robot1 = Robot.RPC('192.168.58.2')
# robot1.LoggerInit(file_path="D://zUP/fairino_log.log")
# robot1.SetLoggerLevel(lvl=1)
# robot1.ResetAllError()
# while True:
#     print("第",i,"轮测试**************")
#     i = i + 1
#     time.sleep(1)
#     # robot1 = Robot.RPC('192.168.58.2')
#     robot1.RobotEnable(0)
#     time.sleep(1)
#     robot1.RobotEnable(1)
#     time.sleep(2)
#     print("jt_cur_pos0:", robot1.robot_state_pkg.jt_cur_pos[0])
#     print("jt_cur_pos1:", robot1.robot_state_pkg.jt_cur_pos[1])
#     print("jt_cur_pos2:", robot1.robot_state_pkg.jt_cur_pos[2])
#     print("jt_cur_pos3:", robot1.robot_state_pkg.jt_cur_pos[3])
#     print("jt_cur_pos4:", robot1.robot_state_pkg.jt_cur_pos[4])
#     print("jt_cur_pos5:", robot1.robot_state_pkg.jt_cur_pos[5])
#     error = robot1.MoveL(desc_pos=DP, tool=0, user=0, vel=60)
#     print("MoveJ return ", error)
#     print("*******************")
#
#
#     time.sleep(1)
#     robot1.RobotEnable(0)
#     time.sleep(1)
#     robot1.RobotEnable(1)
#     time.sleep(2)
#     print("jt_cur_pos0:", robot1.robot_state_pkg.jt_cur_pos[0])
#     print("jt_cur_pos1:", robot1.robot_state_pkg.jt_cur_pos[1])
#     print("jt_cur_pos2:", robot1.robot_state_pkg.jt_cur_pos[2])
#     print("jt_cur_pos3:", robot1.robot_state_pkg.jt_cur_pos[3])
#     print("jt_cur_pos4:", robot1.robot_state_pkg.jt_cur_pos[4])
#     print("jt_cur_pos5:", robot1.robot_state_pkg.jt_cur_pos[5])
#     error = robot1.MoveL(desc_pos=DP1, tool=0, user=0, vel=60)
#     print("MoveJ return ", error)
#     print("*******************")




while True:
    print("第",i,"轮测试**************")
    i = i + 1
    time.sleep(1)
    robot1 = Robot.RPC('192.168.58.2')
    robot1.LoggerInit(file_path="D://zUP/fairino_log.log")
    robot1.SetLoggerLevel(lvl=1)
    robot1.RobotEnable(0)
    time.sleep(1)
    robot1.RobotEnable(1)
    time.sleep(2)
    print("jt_cur_pos0:", robot1.robot_state_pkg.jt_cur_pos[0])
    print("jt_cur_pos1:", robot1.robot_state_pkg.jt_cur_pos[1])
    print("jt_cur_pos2:", robot1.robot_state_pkg.jt_cur_pos[2])
    print("jt_cur_pos3:", robot1.robot_state_pkg.jt_cur_pos[3])
    print("jt_cur_pos4:", robot1.robot_state_pkg.jt_cur_pos[4])
    print("jt_cur_pos5:", robot1.robot_state_pkg.jt_cur_pos[5])
    error = robot1.MoveL(desc_pos=DP, tool=0, user=0, vel=60)
    print("MoveJ return ", error)
    robot1.CloseRPC()
    time.sleep(0.5)
    robot1 = None
    print("*******************")


    time.sleep(1)
    robot2 = Robot.RPC('192.168.58.2')
    robot2.LoggerInit(file_path="D://zUP/fairino_log.log")
    robot2.SetLoggerLevel(lvl=1)
    robot2.RobotEnable(0)
    time.sleep(1)
    robot2.RobotEnable(1)
    time.sleep(2)
    print("jt_cur_pos0:", robot2.robot_state_pkg.jt_cur_pos[0])
    print("jt_cur_pos1:", robot2.robot_state_pkg.jt_cur_pos[1])
    print("jt_cur_pos2:", robot2.robot_state_pkg.jt_cur_pos[2])
    print("jt_cur_pos3:", robot2.robot_state_pkg.jt_cur_pos[3])
    print("jt_cur_pos4:", robot2.robot_state_pkg.jt_cur_pos[4])
    print("jt_cur_pos5:", robot2.robot_state_pkg.jt_cur_pos[5])
    error = robot2.MoveL(desc_pos=DP1, tool=0, user=0, vel=60)
    print("MoveJ return ", error)
    robot2.CloseRPC()
    time.sleep(0.5)
    robot2 = None
    print("*******************")
    #
    # time.sleep(1)
    # robot3 = Robot.RPC('192.168.58.2')
    # print("jt_cur_pos0:", robot3.robot_state_pkg.jt_cur_pos[0])
    # print("jt_cur_pos1:", robot3.robot_state_pkg.jt_cur_pos[1])
    # print("jt_cur_pos2:", robot3.robot_state_pkg.jt_cur_pos[2])
    # print("jt_cur_pos3:", robot3.robot_state_pkg.jt_cur_pos[3])
    # print("jt_cur_pos4:", robot3.robot_state_pkg.jt_cur_pos[4])
    # print("jt_cur_pos5:", robot3.robot_state_pkg.jt_cur_pos[5])
    # error = robot3.MoveJ(joint_pos=JP, tool=0, user=0, vel=60)
    # print("MoveJ return ", error)
    # robot3.CloseRPC()
    # time.sleep(0.5)
    # robot3 = None
    # print("*******************")
    #
    # time.sleep(1)
    # robot4 = Robot.RPC('192.168.58.2')
    # print("jt_cur_pos0:", robot4.robot_state_pkg.jt_cur_pos[0])
    # print("jt_cur_pos1:", robot4.robot_state_pkg.jt_cur_pos[1])
    # print("jt_cur_pos2:", robot4.robot_state_pkg.jt_cur_pos[2])
    # print("jt_cur_pos3:", robot4.robot_state_pkg.jt_cur_pos[3])
    # print("jt_cur_pos4:", robot4.robot_state_pkg.jt_cur_pos[4])
    # print("jt_cur_pos5:", robot4.robot_state_pkg.jt_cur_pos[5])
    # error = robot4.MoveJ(joint_pos=JP1, tool=0, user=0, vel=60)
    # print("MoveJ return ", error)
    # robot4.CloseRPC()
    # time.sleep(0.5)
    # robot4 = None
    # print("*******************")
    #
    # time.sleep(1)
    # robot5 = Robot.RPC('192.168.58.2')
    # print("jt_cur_pos0:", robot5.robot_state_pkg.jt_cur_pos[0])
    # print("jt_cur_pos1:", robot5.robot_state_pkg.jt_cur_pos[1])
    # print("jt_cur_pos2:", robot5.robot_state_pkg.jt_cur_pos[2])
    # print("jt_cur_pos3:", robot5.robot_state_pkg.jt_cur_pos[3])
    # print("jt_cur_pos4:", robot5.robot_state_pkg.jt_cur_pos[4])
    # print("jt_cur_pos5:", robot5.robot_state_pkg.jt_cur_pos[5])
    # error = robot5.MoveJ(joint_pos=JP, tool=0, user=0, vel=60)
    # print("MoveJ return ", error)
    # robot5.CloseRPC()
    # time.sleep(0.5)
    # robot5 = None
    # print("*******************")
    #
    # time.sleep(1)
    # robot6 = Robot.RPC('192.168.58.2')
    # print("jt_cur_pos0:", robot6.robot_state_pkg.jt_cur_pos[0])
    # print("jt_cur_pos1:", robot6.robot_state_pkg.jt_cur_pos[1])
    # print("jt_cur_pos2:", robot6.robot_state_pkg.jt_cur_pos[2])
    # print("jt_cur_pos3:", robot6.robot_state_pkg.jt_cur_pos[3])
    # print("jt_cur_pos4:", robot6.robot_state_pkg.jt_cur_pos[4])
    # print("jt_cur_pos5:", robot6.robot_state_pkg.jt_cur_pos[5])
    # error = robot6.MoveJ(joint_pos=JP1, tool=0, user=0, vel=60)
    # print("MoveJ return ", error)
    # robot6.CloseRPC()
    # time.sleep(0.5)
    # robot6 = None
    # print("*******************")
    #
    #
    # time.sleep(1)
    # robot7 = Robot.RPC('192.168.58.2')
    # print("jt_cur_pos0:", robot7.robot_state_pkg.jt_cur_pos[0])
    # print("jt_cur_pos1:", robot7.robot_state_pkg.jt_cur_pos[1])
    # print("jt_cur_pos2:", robot7.robot_state_pkg.jt_cur_pos[2])
    # print("jt_cur_pos3:", robot7.robot_state_pkg.jt_cur_pos[3])
    # print("jt_cur_pos4:", robot7.robot_state_pkg.jt_cur_pos[4])
    # print("jt_cur_pos5:", robot7.robot_state_pkg.jt_cur_pos[5])
    # error = robot7.MoveJ(joint_pos=JP, tool=0, user=0, vel=60)
    # print("MoveJ return ", error)
    # robot7.CloseRPC()
    # time.sleep(0.5)
    # robot7 = None
    # print("*******************")
    #
    # time.sleep(1)
    # robot8 = Robot.RPC('192.168.58.2')
    # print("jt_cur_pos0:", robot8.robot_state_pkg.jt_cur_pos[0])
    # print("jt_cur_pos1:", robot8.robot_state_pkg.jt_cur_pos[1])
    # print("jt_cur_pos2:", robot8.robot_state_pkg.jt_cur_pos[2])
    # print("jt_cur_pos3:", robot8.robot_state_pkg.jt_cur_pos[3])
    # print("jt_cur_pos4:", robot8.robot_state_pkg.jt_cur_pos[4])
    # print("jt_cur_pos5:", robot8.robot_state_pkg.jt_cur_pos[5])
    # error = robot8.MoveJ(joint_pos=JP1, tool=0, user=0, vel=60)
    # print("MoveJ return ", error)
    # robot8.CloseRPC()
    # time.sleep(0.5)
    # robot8 = None
    # print("*******************")
    #
    # time.sleep(1)
    # robot9 = Robot.RPC('192.168.58.2')
    # print("jt_cur_pos0:", robot9.robot_state_pkg.jt_cur_pos[0])
    # print("jt_cur_pos1:", robot9.robot_state_pkg.jt_cur_pos[1])
    # print("jt_cur_pos2:", robot9.robot_state_pkg.jt_cur_pos[2])
    # print("jt_cur_pos3:", robot9.robot_state_pkg.jt_cur_pos[3])
    # print("jt_cur_pos4:", robot9.robot_state_pkg.jt_cur_pos[4])
    # print("jt_cur_pos5:", robot9.robot_state_pkg.jt_cur_pos[5])
    # error = robot9.MoveJ(joint_pos=JP, tool=0, user=0, vel=60)
    # print("MoveJ return ", error)
    # robot9.CloseRPC()
    # time.sleep(0.5)
    # robot9 = None
    # print("*******************")
    #
    # time.sleep(1)
    # robot10 = Robot.RPC('192.168.58.2')
    # print("jt_cur_pos0:", robot10.robot_state_pkg.jt_cur_pos[0])
    # print("jt_cur_pos1:", robot10.robot_state_pkg.jt_cur_pos[1])
    # print("jt_cur_pos2:", robot10.robot_state_pkg.jt_cur_pos[2])
    # print("jt_cur_pos3:", robot10.robot_state_pkg.jt_cur_pos[3])
    # print("jt_cur_pos4:", robot10.robot_state_pkg.jt_cur_pos[4])
    # print("jt_cur_pos5:", robot10.robot_state_pkg.jt_cur_pos[5])
    # error = robot10.MoveJ(joint_pos=JP1, tool=0, user=0, vel=60)
    # print("MoveJ return ", error)
    # robot10.CloseRPC()
    # time.sleep(0.5)
    # robot10 = None
    # print("*******************")




# movej_test(robot)
# test(robot)
# test0116(robot)
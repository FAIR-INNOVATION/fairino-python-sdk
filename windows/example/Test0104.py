from fairino import Robot
import time
# A connection is established with the robot controller. A successful connection returns a robot object
robot = Robot.RPC('192.168.58.2')

def movej_test(self):
    """机器人MoveJ测试"""
    # JP = [137.678,-97.426,-59.893,-102.467,90.287,15.775]
    # DP = [-329.379,437.127,646.486,-170.238,3.036,32.188]
    #
    # JP1 = [102.325,-95.509,-59.849,-106.768,90.294,15.754]
    # DP1 = [-10.827,524.713,655.736,-172.496,2.409,-3.249]

    JP = [-17.099, -102.71, -114.101, -8.262, 120.908, -18.8]

    JP1 = [-3.349, -102.695, -113.797, -7.265, 121.789, -18.799]


    while True:
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
    flag = 1
    sensor_id = 2
    select = [0, 0, 1, 0, 0, 0] #只启用x轴碰撞守护
    max_threshold = [0.01, 0.01, 5.01, 0.01, 0.01, 0.01]
    min_threshold = [0.01, 0.01, 5.01, 0.01, 0.01, 0.01]

    ft = [1.0, 0.0, 2.0, 0.0, 0.0, 0.0]

    desc_p1 = [-280.5, -474.534, 320.677, 177.986, 1.498, -118.235]
    desc_p2 = [-283.273, -468.668, 172.905, 177.986, 1.498, -118.235]

    safetyMargin = [5, 5, 5, 5, 5, 5]
    robot.SetCollisionStrategy(5, 1000, 140, safetyMargin=safetyMargin)
    rtn = robot.FT_Guard(flag, sensor_id, select, ft, max_threshold, min_threshold)
    print("FT_Guard start rtn " + rtn)
    robot.MoveCart(desc_p1, 0, 0, 20, 100.0)
    robot.MoveCart(desc_p2, 0, 0, 20, 100.0)
    flag = 0
    rtn = robot.FT_Guard(flag, sensor_id, select, ft, max_threshold, min_threshold)
    print("FT_Guard end rtn " + rtn)


def closeRPC(self):
    """CloseRPC功能测试"""
    # robot = Robot.RPC('192.168.58.2')
    # JP = [137.678, -97.426, -59.893, -102.467, 90.287, 15.775]
    # DP = [-329.379, 437.127, 646.486, -170.238, 3.036, 32.188]
    #
    # JP1 = [102.325, -95.509, -59.849, -106.768, 90.294, 15.754]
    # DP1 = [-10.827, 524.713, 655.736, -172.496, 2.409, -3.249]

    JP = [-17.099, -102.71, -114.101, -8.262, 120.908, -18.8]

    JP1 = [-3.349, -102.695, -113.797, -7.265, 121.789, -18.799]

    error = robot.MoveJ(joint_pos=JP, tool=0, user=0, vel=30)
    print("MoveJ return ", error)

    robot.CloseRPC()

    time.sleep(2)
    robot1 = Robot.RPC('192.168.58.2')
    error = robot1.MoveJ(joint_pos=JP1, tool=0, user=0, vel=30)
    print("MoveJ return ", error)

    # while True:

    robot1.CloseRPC()

    time.sleep(2)
    robot2 = Robot.RPC('192.168.58.2')
    error = robot2.MoveJ(joint_pos=JP, tool=0, user=0, vel=30)
    print("MoveJ return ", error)

    robot2.CloseRPC()

    time.sleep(2)
    robot3 = Robot.RPC('192.168.58.2')
    error = robot3.MoveJ(joint_pos=JP1, tool=0, user=0, vel=30)
    print("MoveJ return ", error)

def reset(self):
    """清除Web界面错误"""
    error = robot.ResetAllError()
    print("ResetAllError return ", error)


def SetCollisionStrategy(self):
    flag = 1
    sensor_id = 2
    select = [0, 0, 1, 0, 0, 0]  # 只启用x轴碰撞守护
    max_threshold = [0.01, 0.01, 5.01, 0.01, 0.01, 0.01]
    min_threshold = [0.01, 0.01, 5.01, 0.01, 0.01, 0.01]

    ft = [1.0, 0.0, 2.0, 0.0, 0.0, 0.0]

    desc_p1 = [-280.5, -474.534, 320.677, 177.986, 1.498, -118.235]
    desc_p2 = [-283.273, -468.668, 172.905, 177.986, 1.498, -118.235]

    safetyMargin = [1, 1, 1, 1, 1, 1]
    robot.SetCollisionStrategy(5, 1000, 150,150, safetyMargin=safetyMargin)
    rtn = robot.FT_Guard(flag, sensor_id, select, ft, max_threshold, min_threshold)
    print("FT_Guard start rtn ",rtn)
    robot.MoveCart(desc_p1, 0, 0, 20, 100.0)
    robot.MoveCart(desc_p2, 0, 0, 20, 100.0)
    flag = 0
    rtn = robot.FT_Guard(flag, sensor_id, select, ft, max_threshold, min_threshold)
    print("FT_Guard end rtn ",rtn)

def TRSF(self,enable):
    rtn = 0
    startdescPose = [-226.699, -501.969, 264.638, -174.973, 5.852, 143.301]
    startjointPos = [52.850, -84.327, 102.163, -112.843, -84.131, 0.063]

    enddescPose = [-226.702, -501.973, 155.833, -174.973, 5.852, 143.301]
    endjointPos = [52.850, -77.596, 111.785, -129.196, -84.131, 0.062]

    exaxisPos = [0, 0, 0, 0]
    offdese = [0, 0, 0, 0, 0, 0]

    if enable:
        robot.ToolTrsfStart(1)
        rtn = robot.MoveJ(startjointPos, 0, 0, startdescPose)
        print("rtn is ", rtn)
        rtn = robot.MoveJ(endjointPos, 0, 0, enddescPose)
        print("rtn is ", rtn)
        robot.ToolTrsfEnd()
    else:
        rtn = robot.MoveJ(startjointPos, 0, 0, startdescPose)
        print("rtn is ", rtn)
        rtn = robot.MoveJ(endjointPos, 0, 0, enddescPose)
        print("rtn is ", rtn)



movej_test(robot)
# test(robot)
# closeRPC(robot)
# reset(robot)
# SetCollisionStrategy(robot)


# TRSF(robot,0)
# TRSF(robot,1)
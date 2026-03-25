
from time import sleep
import time
from fairino import Robot

# 与机器人控制器建立连接
robot = Robot.RPC('192.168.58.2')


def TestServoJUDP(self):
    # 设置简单的回调
    def callback(src_type, count, cmd_id, data_len, content):
        print("回调函数: cmd_id={} count={} data_len={} content={}".format(cmd_id, count, data_len, content))
        return 0

    robot.SetUDPCmdRpyCallback(callback)
    # # 初始化关节位置和外部轴位置
    j= [105, -108, 74, -66, -88.893, -1.621]
    offset_pos = [0, 0, 0, 0, 0, 0]
    epos = [0, 0, 0, 0]
    # # 移动到初始位置
    result=robot.MoveJ(joint_pos=j, tool=0, user=0, vel=100, acc=100, ovl=100,
                exaxis_pos=epos, blendT=-1, offset_flag=0, offset_pos=offset_pos)
    print("MoveJ返回结果: {}".format(result))
    vel = 0.0
    acc = 0.0
    cmdT = 0.016
    filterT = 0.0
    gain = 0.0
    flag = 0
    dt = 0.1
    cmdID = 0

    # 获取当前关节位置
    ret, j = robot.GetActualJointPosDegree(flag)
    if ret != 0:
        print(f"GetActualJointPosDegree errcode:{ret}")
    while 1:
        count = 300
        result = robot.ServoMoveStart(cmdType=1)
        print("ServoMoveStart返回结果: {}".format(result))
        while count > 0:
            result = robot.ServoJ(joint_pos=j, axisPos=epos, acc=acc, vel=vel, cmdT=cmdT,
                         filterT=filterT, gain=gain, id=cmdID, cmdType=1)
            j[0] += dt
            j[1] += dt
            j[2] += dt
            j[3] += dt
            j[4] += dt
            j[5] += dt
            # epos[0] += dt
            count -= 1
            print("返回结果: {}".format(result))
            time.sleep(0.01)
        result = robot.ServoMoveEnd(cmdType=1)
        print("ServoMoveEnd返回结果: {}".format(result))

        count = 300
        result = robot.ServoMoveStart(cmdType=1)
        print("ServoMoveStart返回结果: {}".format(result))
        while count > 0:
            result = robot.ServoJ(joint_pos=j, axisPos=epos, acc=acc, vel=vel, cmdT=cmdT,
                         filterT=filterT, gain=gain, id=cmdID, cmdType=1)
            j[0] -= dt
            j[1] -= dt
            j[2] -= dt
            j[3] -= dt
            j[4] -= dt
            j[5] -= dt
            # epos[0] -= dt
            count -= 1
            print("ServoJ返回结果: {}".format(result))
            time.sleep(0.01)
        result = robot.ServoMoveEnd(cmdType=1)
        print("ServoMoveEnd返回结果: {}".format(result))
    robot.CloseRPC()
    return 0


def TestServoJTUDP(self):
    # 设置简单的回调
    def callback(src_type, count, cmd_id, data_len, content):
        print("回调函数: cmd_id={} count={} data_len={} content={}".format(cmd_id, count, data_len, content))
        return 0

    robot.SetUDPCmdRpyCallback(callback)
    while True:
        # 初始化关节位置和外部轴位置
        j = [0, -90, 90, 0, 0, 0]
        epos = [0, 0, 0, 0]
        offset_pos = [0, 0, 0, 0, 0, 0]

        # 移动到初始位置
        robot.MoveJ(joint_pos=j, tool=0, user=0, vel=100, acc=100, ovl=100,
                    exaxis_pos=epos, blendT=-1, offset_flag=0, offset_pos=offset_pos)
        time.sleep(3)
        # 开启拖动示教
        result=robot.DragTeachSwitch(1)
        print("DragTeachSwitch返回结果: {}".format(result))
        torques = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        # 获取关节力矩
        ret, torques = robot.GetJointTorques(flag=1)
        if ret != 0:
            print(f"GetJointTorques errcode:{ret}")

        count = 100
        result = robot.ServoJTStart(cmdType=1)
        print("ServoJTStart返回结果: {}".format(result))
        # 正向力矩控制
        while True:
            torques[0] = 0.03
            result = robot.ServoJT(
                torque=torques,
                interval=0.001,
                checkFlag=0,
                jPowerLimit=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                jVelLimit=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                cmdType=1
            )
            print("返回结果: {}".format(result))
            time.sleep(1)

            ret, pkg = robot.GetRobotRealTimeState()
            if pkg.jt_cur_pos[0] > 30:
                break

        # 反向力矩控制
        while True:
            torques[0] = -0.03
            result = robot.ServoJT(
                    torque=torques,
                    interval=0.001,
                    checkFlag=0,
                    jPowerLimit=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                    jVelLimit=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                    cmdType=1
                )
            print("返回结果: {}".format(result))
            time.sleep(1)

            ret, pkg = robot.GetRobotRealTimeState()
            if pkg.jt_cur_pos[0] < 0:
                break

        # 结束力矩控制并关闭拖动示教
        result = robot.ServoJTEnd(cmdType=1)
        print("ServoJTEnd返回结果: {}".format(result))
        result = robot.DragTeachSwitch(0)
        print("DragTeachSwitch返回结果: {}".format(result))

    robot.CloseRPC()
    return 0


def TestSendUDPFrame(self):
    # 设置简单的回调
    def callback(src_type, count, cmd_id, data_len, content):
        print("收到回复: cmd_id={} count={} data_len={} content={}".format(cmd_id, count, data_len, content))
        return 0

    robot.SetUDPCmdRpyCallback(callback)

    rtn = robot.SendUDPFrame("/f/bIII20III303III7IIIMode(0)III/b/f")
    print(f"SendUDPFrame Mode(0) rtn is {rtn}")

    time.sleep(1)

    rtn = robot.SendUDPFrame("/f/bIII21III303III7IIIMode(1)III/b/f")
    print(f"SendUDPFrame Mode(1) rtn is {rtn}")

    time.sleep(1)

    rtn = robot.SendUDPFrame(
        "/f/bIII49III201III184IIIMoveJ(-15.625, -82.680, 101.654, -110.950, -88.290, 0.017, -383.012, -2.325, 242.655, -178.024, 1.710, 74.416, 0, 0, 100, 100, 100, 0.000, 0.000, 0.000, 0.000, -1, 0, 0, 0, 0, 0, 0, 0)III/b/f")
    print(f"SendUDPFrame MoveJ(-15.625) rtn is {rtn}")

    time.sleep(1)

    rtn = robot.SendUDPFrame(
        "/f/bIII48III203III199IIIMoveL(-75.622, -82.680, 101.654, -110.950, -88.290, 0.017, -193.537, 330.525, 242.657, -178.024, 1.710, 14.420, 0, 0, 100, 100, 100, -1, 0, 0.000, 0.000, 0.000, 0.000, 0, 0, 0, 0, 0, 0, 0, 0, 100, 0)III/b/f")
    print(f"SendUDPFrame MoveL(-75.622) rtn is {rtn}")

    time.sleep(1)

    rtn = robot.SendUDPFrame("/f/bIII4III905III20IIIGetSoftwareVersion()III/b/f")
    print(f"SendUDPFrame GetSoftwareVersion() rtn is {rtn}")

    time.sleep(1)

    # 发送UDP帧数据校验测试
    rtn = robot.SendUDPFrame("/f/bIII20III303III7IIIMode(0)III/b/f")
    print(f"SendUDPFrame rtn is {rtn}")

    rtn = robot.SendUDPFrame("III20III303III7IIIMode(0)III/b/f")
    print(f"SendUDPFrame rtn is {rtn}")

    rtn = robot.SendUDPFrame("/f/bIII20III303III7IIIMode(0)")
    print(f"SendUDPFrame rtn is {rtn}")

    rtn = robot.SendUDPFrame("/f/bIII20III303III6IIIMode(0)III/b/f")
    print(f"SendUDPFrame rtn is {rtn}")

    rtn = robot.SendUDPFrame("/f/b|||20|||303|||7|||Mode(0)|||/b/f")
    print(f"SendUDPFrame rtn is {rtn}")

    rtn = robot.SendUDPFrame("/f/bII20II303II7IIMode(0)II/b/f")
    print(f"SendUDPFrame rtn is {rtn}")

    robot.CloseRPC()
    time.sleep(1)

TestServoJUDP(robot)
#TestServoJTUDP(robot)
# TestSendUDPFrame(robot)

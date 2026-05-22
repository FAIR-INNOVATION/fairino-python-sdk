from time import sleep
import time
from fairino import Robot

# 与机器人控制器建立连接
robot = Robot.RPC('192.168.58.2')


# 定义回调函数
def udp_frame_callback(src_type, count, cmd_id, data_len, content):
    """UDP命令响应回调函数"""
    print(f"回调函数: cmd_id={cmd_id} count={count} data_len={data_len} content={content}")
    return 0


def ServoMITtest(self):
    # 设置UDP命令响应回调
    robot.SetUDPCmdRpyCallback(udp_frame_callback)

    while True:
        # 复位所有错误
        robot.ResetAllError()
        time.sleep(0.5)

        # 初始化参数数组
        posGain = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        desPos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        velGain = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        desVel = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        torques = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        # 获取关节力矩
        rtn, torques = robot.GetJointTorques(flag=1)
        print(f"GetJointTorques rtn: {rtn}")
        print("111111")

        # 启动Servo MIT模式
        rtn = robot.ServoMITStart(0)
        print(f"ServoMITStart rtn: {rtn}")

        # 开启拖动示教
        rtn = robot.DragTeachSwitch(1)
        print(f"DragTeachSwitch rtn: {rtn}")

        intev = 0.008

        # 正向运动：第6轴正向力矩，直到角度超过30度
        while True:
            torques[5] = 0.03
            rtn = robot.ServoMIT(posGain, desPos, velGain,
                                 desVel, torques, intev, comType=0)
            print(f"ServoMIT call rtn is {rtn}")
            time.sleep(0.001)  # 1ms

            rtn, pkg = robot.GetRobotRealTimeState()
            print(f"pkg.jt_cur_pos[5]: {pkg.jt_cur_pos[5]}")

            if pkg.jt_cur_pos[5] > 30:
                break

        # 反向运动：第6轴负向力矩，直到角度小于0度
        while True:
            torques[5] = -0.03
            rtn = robot.ServoMIT(posGain, desPos, velGain,
                                 desVel, torques, intev, comType=0)
            print(f"ServoMIT call rtn is {rtn}")
            time.sleep(0.001)  # 1ms

            rtn, pkg = robot.GetRobotRealTimeState()
            print(f"pkg.jt_cur_pos[5]: {pkg.jt_cur_pos[5]}")

            if pkg.jt_cur_pos[5] < 0:
                break

        # 关闭拖动示教
        rtn = robot.DragTeachSwitch(0)
        print(f"DragTeachSwitch off rtn: {rtn}")

        # 结束Servo MIT模式
        rtn = robot.ServoMITEnd(0)
        print(f"ServoMITEnd rtn: {rtn}")


# 调用测试函数
ServoMITtest(robot)
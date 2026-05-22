from fairino import Robot
import time


def main():
    # 与机器人控制器建立连接
    robot = Robot.RPC('192.168.58.2')
    time.sleep(0.5)  # 等待连接和数据接收

    # 初始化关节速度数组和扩展轴速度数组
    joint_vel = [10.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    exis_vel = [0.0, 0.0, 0.0, 0.0]
    acc = 0.0
    vel = 0.0
    cmdT = 0.008
    filterT = 0.0
    gain = 0.0
    cnt = 0

    # 循环调用ServoJV，共200次
    while cnt < 200:
        rtn = robot.ServoJV(joint_vel=joint_vel, exis_vel=exis_vel, acc=acc, vel=vel,
                            cmdT=cmdT, filterT=filterT, gain=gain)
        print(f"ServoJV rtn is {rtn}")
        cnt += 1

    # 关闭连接
    robot.CloseRPC()


# 调用测试函数
main()
from time import sleep
import time
from fairino import Robot

# 与机器人控制器建立连接
robot = Robot.RPC('192.168.58.2')

def TestOriginPointWeave(self):
    time.sleep(2)
    # 初始化关节位置、外部轴和偏移
    j = [39.886, -98.580, -124.032, -47.393, 90.000, 40.842]
    epos1 = [0, 0, 0, 0]
    offset_pos = [0, 0, 0, 0, 0, 0]
    epos2 = [5, 0.000, 0.000, 0.000]
    # 参考点位置 [x, y, z, rx, ry, rz]
    refPoint = [400.021, 300.022, 299.996, 179.997, -0.003, -90.956]

    # 移动到起始位置
    # robot.MoveJ(joint_pos=j, tool=1, user=0, vel=100, acc=100, ovl=100,
    #             exaxis_pos=epos1, blendT=-1, offset_flag=0, offset_pos=offset_pos)

    rtn = 0
    robot.LaserTrackingSensorConfig("192.168.58.20", 5020)
    robot.LaserTrackingSensorSamplePeriod(20)
    robot.LoadPosSensorDriver(101)

    # 加载 UDP 驱动
    robot.ExtDevLoadUDPDriver()

    # 设置外部轴命令完成时间
    rtn = robot.SetExAxisCmdDoneTime(5000.0)
    print(f"SetExAxisCmdDoneTime rtn is {rtn}")

    # 使能外部轴 1 和 2
    rtn = robot.ExtAxisServoOn(1, 1)
    print(f"ExtAxisServoOn axis id 1 rtn is {rtn}")
    rtn = robot.ExtAxisServoOn(2, 1)
    print(f"ExtAxisServoOn axis id 2 rtn is {rtn}")
    time.sleep(2)

    # 设置外部轴回零
    robot.ExtAxisSetHoming(1, 0, 10, 2)
    robot.LaserTrackingLaserOnOff(1)

    # 1---不带扩展轴
    robot.LaserTrackingTrackOnOff(1, 4)
    time.sleep(0.2)
    # 启动定点摆动
    robot.OriginPointWeaveStart(0, 0, refPoint, 10)
    robot.MoveStationary()  # 执行固定运动（假设该方法存在）
    robot.OriginPointWeaveEnd()
    robot.LaserTrackingTrackOnOff(0, 4)

    time.sleep(2)  # 等待2秒

    # 2----带扩展轴
    robot.ExtAxisMove(epos1, 100, -1)
    robot.LaserTrackingTrackOnOff(1, 4)
    # 启动定点摆动
    robot.OriginPointWeaveStart(0, 0, refPoint, 20)
    robot.ExtAxisMove(epos2, 100, -1)
    robot.OriginPointWeaveEnd()
    robot.LaserTrackingTrackOnOff(0, 4)

    # 关闭连接
    robot.CloseRPC()
    time.sleep(1)

TestOriginPointWeave(robot)
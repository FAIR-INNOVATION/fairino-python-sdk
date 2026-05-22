from time import sleep
from fairino import Robot
import time

# 与机器人控制器建立连接，连接成功返回一个机器人对象
robot = Robot.RPC('192.168.58.2')


def testLsaerWeld():
    robot.ExtDevLoadUDPDriver()
    time.sleep(1)

    robot.SetLaserWeldingParam(num = 3, scanSpeed = 2000, scanWidth = 3, peakPower = 1500, dutyCycle = 100, Freq = 1000, io_type=1)
    robot.SetLaserWeldingStartExtDoNum(ctrlModeDONum=1)

    robot.Mode(0)
    time.sleep(1)

    desc_pos1 = [-303.721, -206.960, 297.105, 152.209, 19.857, 109.166]
    desc_pos2 = [-301.575, -254.888, 284.786, 155.919, 26.946, 111.629]
    desc_safe = [-344.386, -280.830, 435.073, 173.835, 15.333, 124.931]
    jointPos1 = [9.827, -99.740, 120.088, -78.900, -77.241, -17.904]
    jointPos2 = [15.251, -96.456, 120.138, -84.664, -68.542, -17.843]
    jointSafe = [19.142, -98.078, 101.493, -83.078, -77.070, -17.794]


    error = robot.MoveL(desc_pos=desc_pos1,joint_pos=jointPos1, tool=1, user=0, vel=100, ovl= 2, acc=100)
    print("MoveL return:", error)
    robot.SetLaserWeldingStartEnd(1, io_type=1, max_waittime=10000)

    error = robot.MoveL(desc_pos=desc_pos2, joint_pos=jointPos2, tool=1, user=0, vel=100, ovl= 2, acc=100)
    print("MoveL return:", error)
    robot.SetLaserWeldingStartEnd(0, io_type=1, max_waittime=10000)

    error = robot.MoveL(desc_pos=desc_safe, joint_pos=jointSafe, tool=1, user=0, vel=100, ovl= 2, acc=100)
    print("MoveL return:", error)

    robot.Mode(1)
    time.sleep(1)

    # 关闭连接
    robot.CloseRPC()
    time.sleep(1)

# 调用测试函数
testLsaerWeld()


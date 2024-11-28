from fairino import Robot
import time

# 与机器人控制器建立连接，连接成功返回一个机器人对象
robot = Robot.RPC('192.168.58.2')


def TestTractorMove(self):
    """小车运动"""
    robot.ExtDevSetUDPComParam("192.168.58.2", 2021, 2, 50, 5, 50, 1, 50, 10)
    robot.ExtDevLoadUDPDriver()
    robot.ExtAxisParamConfig(1, 0, 0, 50000, -50000, 1000, 1000, 6.280, 16384, 200, 0, 0, 0)
    robot.ExtAxisParamConfig(2, 0, 0, 50000, -50000, 1000, 1000, 6.280, 16384, 200, 0, 0, 0)
    robot.SetAxisDHParaConfig(5, 0, 0, 0, 0, 0, 0, 0, 0)

    # robot.TractorEnable(False)
    # time.sleep(2)
    # robot.TractorEnable(True)
    # time.sleep(2)
    # robot.TractorHoming()
    # time.sleep(2)
    robot.TractorMoveL(100, 20)
    time.sleep(5)
    robot.TractorMoveL(-100, 20)
    time.sleep(5)
    robot.TractorMoveC(50, 60, 20)
    time.sleep(5)
    robot.TractorMoveC(50, -60, 20)

def TestTractorMoveStop(self):
    """小车运动中停止"""
    robot.ExtDevSetUDPComParam("192.168.58.2", 2021, 2, 50, 5, 50, 1, 50, 10)
    robot.ExtDevLoadUDPDriver()
    robot.ExtAxisParamConfig(1, 0, 0, 50000, -50000, 1000, 1000, 6.280, 16384, 200, 0, 0, 0)
    robot.ExtAxisParamConfig(2, 0, 0, 50000, -50000, 1000, 1000, 6.280, 16384, 200, 0, 0, 0)
    robot.SetAxisDHParaConfig(5, 0, 0, 0, 0, 0, 0, 0, 0)

    robot.TractorEnable(False)
    time.sleep(2)
    robot.TractorEnable(True)
    time.sleep(2)
    robot.TractorHoming()
    time.sleep(2)
    robot.TractorMoveL(100, 20)
    time.sleep(5)
    robot.TractorMoveL(-100, 20)
    time.sleep(5)
    robot.TractorMoveC(300, 90, 20)
    time.sleep(4)
    error = robot.TractorStop()
    print("TractorStop return ", error)

def TestWeldmechineMode(self):
    """焊机控制模式切换"""
    error = robot.ExtDevSetUDPComParam("192.168.58.88", 2021, 2, 50, 5, 50, 1, 50, 10)
    print("ExtDevSetUDPComParam return ", error)
    error = robot.ExtDevLoadUDPDriver()
    print("ExtDevLoadUDPDriver return ", error)

    robot.SetWeldMachineCtrlModeExtDoNum(DONum=17)
    robot.SetWeldMachineCtrlMode(mode=0)
    robot.SetWeldMachineCtrlModeExtDoNum(DONum=18)
    robot.SetWeldMachineCtrlMode(mode=0)
    robot.SetWeldMachineCtrlModeExtDoNum(DONum=19)
    robot.SetWeldMachineCtrlMode(mode=0)

    error = robot.SetWeldMachineCtrlModeExtDoNum(DONum=17)
    print("SetWeldMachineCtrlModeExtDoNum return ", error)
    for  i  in  range(0,5):
        error = robot.SetWeldMachineCtrlMode(mode=0)
        print("SetWeldMachineCtrlMode return ", error)
        time.sleep(0.5)
        error = robot.SetWeldMachineCtrlMode(mode=1)
        print("SetWeldMachineCtrlMode return ", error)
        time.sleep(0.5)

    error = robot.SetWeldMachineCtrlModeExtDoNum(DONum=18)
    print("SetWeldMachineCtrlModeExtDoNum return ", error)
    for  i  in  range(0,5):
        error = robot.SetWeldMachineCtrlMode(mode=0)
        print("SetWeldMachineCtrlMode return ", error)
        time.sleep(0.5)
        error = robot.SetWeldMachineCtrlMode(mode=1)
        print("SetWeldMachineCtrlMode return ", error)
        time.sleep(0.5)

    error = robot.SetWeldMachineCtrlModeExtDoNum(DONum=19)
    print("SetWeldMachineCtrlModeExtDoNum return ", error)
    for  i  in  range(0,5):
        error = robot.SetWeldMachineCtrlMode(mode=0)
        print("SetWeldMachineCtrlMode return ", error)
        time.sleep(0.5)
        error = robot.SetWeldMachineCtrlMode(mode=1)
        print("SetWeldMachineCtrlMode return ", error)
        time.sleep(0.5)

TestTractorMove(robot)
# TestTractorMoveStop(robot)
# TestWeldmechineMode(robot)
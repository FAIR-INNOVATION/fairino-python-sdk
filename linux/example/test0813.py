from time import sleep
from fairino import Robot
# A connection is established with the robot controller. A successful connection returns a robot object
# 与机器人控制器建立连接，连接成功返回一个机器人对象
robot = Robot.RPC('192.168.58.2')
import time

def testSucker(self):
    robot.OpenLuaUpload("C://项目/外设SDK/CtrlDev_sucker.lua")
    time.sleep(2)
    robot.UnloadCtrlOpenLUA(1)
    robot.LoadCtrlOpenLUA(1)
    time.sleep(1)
    ctrl = bytearray(20)
    ctrl[0] = 1
    robot.SetSuckerCtrl(0, 1, ctrl)
    for i in range(100):
        rtn, state, press_value, error = robot.GetSuckerState(1)
        print(f"sucker1 state is {state}, pressValue is {press_value}, error num is {error}")
        rtn, state, press_value, error = robot.GetSuckerState(12)
        print(f"sucker12 state is {state}, pressValue is {press_value}, error num is {error}")
        time.sleep(0.1)
    ret = robot.WaitSuckerState(1, 1, 100)
    print(f"WaitSuckerState result is {ret}")
    ctrl[0] = 3
    robot.SetSuckerCtrl(1, 1, ctrl)
    robot.SetSuckerCtrl(12, 1, ctrl)
    robot.CloseRPC()

def testFieldBusBoard(self):
    robot.OpenLuaUpload("D://zUP/外设/CtrlDev_field.lua")
    time.sleep(2)
    robot.SetCtrlOpenLUAName(3,"CtrlDev_field.lua")
    robot.UnloadCtrlOpenLUA(3)
    robot.LoadCtrlOpenLUA(3)
    time.sleep(8)

    rtn,type_, version, conn_state = robot.GetFieldBusConfig()
    print(f"type is {type_}, version is {version}, connState is {conn_state}")

    # Write digital outputs
    ctrl = [1, 0, 1]  # DO0=1, DO1=0, DO2=1
    robot.FieldBusSlaveWriteDO(0, 3, ctrl)

    # Write analog output
    ctrl_ao = [0x1000]  # AO2 = 0x1000
    robot.FieldBusSlaveWriteAO(2, 1, ctrl_ao)

    for i in range(100):
        rtn,di = robot.FieldBusSlaveReadDI(0, 4)
        print(f"DI0 is {di[0]}, DI1 is {di[1]}, DI2 is {di[2]}, DI3 is {di[3]}")

        rtn, ai = robot.FieldBusSlaveReadAI(0, 3)
        print(f"AI0 is {ai[0]}, AI1 is {ai[1]}, AI2 is {ai[2]}")

        time.sleep(0.01)
    ret = robot.FieldBusSlaveWaitDI(0, 1, 100)
    print(f"FieldBusSlaveWaitDI result is {ret}")

    ret = robot.FieldBusSlaveWaitAI(0, 0, 400.00, 100)
    print(f"FieldBusSlaveWaitAI result is {ret}")

    robot.CloseRPC()


def testSetSuckerCtrl(self):
    ctrl = bytearray(20)
    for i in range(5):
        print(f"------------------第 {i + 1} 次测试------------------")
        ctrl[0] = 1
        robot.SetSuckerCtrl(0, 1, ctrl)
        print("sucker broadcast start")
        time.sleep(3)

        ctrl[0] = 3
        robot.SetSuckerCtrl(0, 1, ctrl)
        print("sucker broadcast stop")
        time.sleep(3)

        ctrl[0] = 2
        robot.SetSuckerCtrl(1, 1, ctrl)
        robot.SetSuckerCtrl(12, 1, ctrl)
        print("sucker unicast start")
        time.sleep(2)

        ctrl[0] = 3
        robot.SetSuckerCtrl(1, 1, ctrl)
        robot.SetSuckerCtrl(12, 1, ctrl)
        print("sucker unicast stop")
        time.sleep(2)
    robot.CloseRPC()

def testGetSuckerState(self):
    ctrl = bytearray(20)
    ctrl[0] = 1
    robot.SetSuckerCtrl(0, 1, ctrl)
    for i in range(100):
        rtn, state1, press_value1, error1 = robot.GetSuckerState(1)
        print(f"sucker1 state is {state1}, pressValue is {press_value1}, error num is {error1}")
        rtn, state12, press_value12, error12 = robot.GetSuckerState(12)
        print(f"sucker12 state is {state12}, pressValue is {press_value12}, error num is {error12}")
        time.sleep(0.1)

    ctrl[0] = 3
    robot.SetSuckerCtrl(0, 1, ctrl)

    robot.CloseRPC()

def testWaitSuckerState(self):
    ctrl = [0] * 20
    state = 0
    pressValue = 0
    error = 0
    ctrl[0] = 1
    robot.SetSuckerCtrl(0, 1, ctrl)
    for i in range(100):
        rtn, state, pressValue, error = robot.GetSuckerState(1)
        print(f"sucker1 state is {state}")
        rtn, state, pressValue, error = robot.GetSuckerState(12)
        print(f"sucker12 state is {state}")
        time.sleep(0.1)

    ret = robot.WaitSuckerState(1, 1, 100)
    print(f"WaitSuckerState1 result is {ret}")
    ret = robot.WaitSuckerState(12, 1, 100)
    print(f"WaitSuckerState12 result is {ret}")
    ctrl[0] = 3
    robot.SetSuckerCtrl(0, 1, ctrl)

    robot.CloseRPC()

def testGetFieldBusConfig(self):
    type = 0
    version = 0
    connState = 0
    ctrl = [0] * 8
    ctrlAO = [0] * 8
    DI = [0] * 8
    AI = [0] * 8
    rtn,type, version, connState = robot.GetFieldBusConfig()
    print(f"type is {type}, version is {version}, connState is {connState}")

    robot.CloseRPC()

def testFieldBusSlaveWriteDOAO(self):
    type = 0
    version = 0
    connState = 0
    ctrl = [0] * 8
    ctrlAO = [0] * 8
    DI = [0] * 8
    AI = [0] * 8
    ctrl[0] = 1
    ctrl[1] = 0
    ctrl[2] = 0
    ctrl[3] = 1
    robot.FieldBusSlaveWriteDO(0, 4, ctrl)
    ctrlAO[0] = 0x1010
    robot.FieldBusSlaveWriteAO(2, 1, ctrlAO)
    robot.CloseRPC()


def testFieldBusSlaveReadDIAI_waitDIAI(self):
    DI = [0] * 8
    AI = [0] * 8
    for i in range(100):
        rtn, DI = robot.FieldBusSlaveReadDI(0, 4)
        print(f"DI0 is {DI[0]}, DI1 is {DI[1]}, DI2 is {DI[2]}, DI3 is {DI[3]}")
        rtn, AI = robot.FieldBusSlaveReadAI(0, 3)
        print(f"AI0 is {AI[0]}, AI1 is {AI[1]}, AI2 is {AI[2]}")
        time.sleep(0.1)
    ret = robot.FieldBusSlaveWaitDI(0, 1, 100)
    print(f"FieldBusSlaveWaitDI result is {ret}")
    ret = robot.FieldBusSlaveWaitAI(0, 0, 400.00, 100)
    print(f"FieldBusSlaveWaitAI result is {ret}")
    robot.CloseRPC()


def testSuckerMove(self):
    ctrl = [0] * 20
    state = 0
    pressValue = 0
    error = 0

    # Load control program
    robot.OpenLuaUpload("D://zUP/外设/CtrlDev_sucker.lua")
    time.sleep(2)
    robot.SetCtrlOpenLUAName(1,"CtrlDev_sucker.lua")
    robot.UnloadCtrlOpenLUA(1)
    robot.LoadCtrlOpenLUA(1)
    time.sleep(1)

    # Define positions as lists
    j1 = [76.558, -81.447, 132.913, -145.499, -92.762, -0.485]
    desc_pos1 = [-2.659, -429.194, 170.829, -175.985, -2.789, 166.848]

    j2 = [76.559, -90.243, 128.285, -132.076, -92.762, -0.485]
    desc_pos2 = [-2.658, -429.198, 241.123, -175.985, -2.789, 166.848]
    offset_pos = [0, 0, 0, 0, 0, 0]
    epos = [0, 0, 0, 0]

    # Movement parameters
    tool = 0
    user = 0
    vel = 100.0
    acc = 100.0
    ovl = 100.0
    blendT = -1.0
    blendR = 0.0
    flag = 0
    search = 0

    robot.SetSpeed(20)

    while True:
        # Move to first position
        rtn = robot.MoveJ(j1, tool=0, user=0, vel=100)

        # Control sucker in broadcast mode with maximum suction
        ctrl[0] = 2
        robot.SetSuckerCtrl(0, 1, ctrl)

        # Monitor sucker 1 and 12 status
        for i in range(20):
            rtn,state, pressValue, error = robot.GetSuckerState(1)
            print(f"sucker1 state is {state}, pressValue is {pressValue}, error num is {error}")
            rtn,state, pressValue, error = robot.GetSuckerState(12)
            print(f"sucker12 state is {state}, pressValue is {pressValue}, error num is {error}")
            time.sleep(0.1)

        # Wait for sucker 1 to grab object
        ret = robot.WaitSuckerState(1, 1, 100)
        print(f"WaitSuckerState result is {ret}")

        # Move to second position
        rtn = robot.MoveJ(j2, tool=0, user=0, vel=100)

        if ret == 0:
            print("sucker1 吸附到物体 (Object grabbed)")
        else:
            print("sucker1 未吸附到物体 (No object grabbed)")
            continue

        # Turn off suckers 1 and 12 in unicast mode
        ctrl[0] = 3
        robot.SetSuckerCtrl(1, 1, ctrl)
        robot.SetSuckerCtrl(12, 1, ctrl)

        time.sleep(1)

    robot.CloseRPC()


# testSucker(robot)
# testSetSuckerCtrl(robot)
# testGetSuckerState(robot)
# testWaitSuckerState(robot)
# testSuckerMove(robot)

# testGetFieldBusConfig(robot)
# testFieldBusSlaveWriteDOAO(robot)
# testFieldBusSlaveReadDIAI_waitDIAI(robot)
# testFieldBusBoard(robot)
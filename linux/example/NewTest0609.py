from fairino import Robot
import time
# A connection is established with the robot controller. A successful connection returns a robot object
import time
import threading

# 与机器人控制器建立连接，连接成功返回一个机器人对象
robot = Robot.RPC('192.168.58.2')

def TestRobotCtrl(self):
    '''机器人控制'''
    error,version = robot.GetSDKVersion()
    print(f"SDK version: {version}")
    error,ip = robot.GetControllerIP()
    print(f"controller ip: {ip}")

    robot.Mode(1)
    time.sleep(1)
    robot.DragTeachSwitch(state=1)
    time.sleep(1)
    error,state = robot.IsInDragTeach()
    print(f"drag state: {state}")
    time.sleep(3)
    robot.DragTeachSwitch(state=0)
    time.sleep(1)
    error,state = robot.IsInDragTeach()
    print(f"drag state: {state}")
    time.sleep(3)
    robot.RobotEnable(0)
    time.sleep(3)
    robot.RobotEnable(1)
    robot.Mode(0)
    time.sleep(1)
    robot.Mode(1)
    # print("\nPress any key to exit!")
    robot.CloseRPC()


def TestGetVersions(self):
    # Get software versions
    rtn,robotModel, webversion, controllerVersion = robot.GetSoftwareVersion()
    print(f"Getsoftwareversion rtn is: {rtn}")
    print(f"robotmodel is: {robotModel}, webversion is: {webversion}, controllerVersion is: {controllerVersion}\n")

    # Get hardware versions
    rtn,ctrlBoxBoardversion, driver1version, driver2version,driver3version, driver4version, driver5version,driver6version, endBoardversion = robot.GetHardwareversion()
    print(f"GetHardwareversion rtn is: {rtn}")
    print(f"GetHardwareversion get hardware version is: {ctrlBoxBoardversion}, {driver1version}, {driver2version}, "
          f"{driver3version}, {driver4version}, {driver5version}, {driver6version}, {endBoardversion}\n")

    # Get firmware versions
    rtn,ctrlBoxBoardversion, driver1version, driver2version,driver3version, driver4version, driver5version,driver6version, endBoardversion = robot.GetFirmwareVersion()
    print(f"GetFirmwareversion rtn is: {rtn}")
    print(f"GetFirmwareversion get firmware version is: {ctrlBoxBoardversion}, {driver1version}, {driver2version}, "
          f"{driver3version}, {driver4version}, {driver5version}, {driver6version}, {endBoardversion}\n")

    # print("\nPress any key to exit!")

    robot.CloseRPC()


def TestJOG(self):
    # Test JOG in different modes for each axis
    for i in range(6):
        robot.StartJOG(0, i + 1, 0, 20.0, 20.0, 30.0)
        time.sleep(1)
        robot.ImmStopJOG()
        time.sleep(1)

    for i in range(6):
        robot.StartJOG(2, i + 1, 0, 20.0, 20.0, 30.0)
        time.sleep(1)
        robot.ImmStopJOG()
        time.sleep(1)

    for i in range(6):
        robot.StartJOG(4, i + 1, 0, 20.0, 20.0, 30.0)
        time.sleep(1)
        robot.StopJOG(5)
        time.sleep(1)

    for i in range(6):
        robot.StartJOG(8, i + 1, 0, 20.0, 20.0, 30.0)
        time.sleep(1)
        robot.StopJOG(9)
        time.sleep(1)

    robot.CloseRPC()


def TestMove(self):

    # Define joint positions
    j1 = [-11.904, -99.669, 117.473, -108.616, -91.726, 74.256]
    j2 = [-45.615, -106.172, 124.296, -107.151, -91.282, 74.255]
    j3 = [-29.777, -84.536, 109.275, -114.075, -86.655, 74.257]
    j4 = [-31.154, -95.317, 94.276, -88.079, -89.740, 74.256]

    # Define Cartesian positions
    desc_pos1 = [-419.524, -13.000, 351.569, -178.118, 0.314, 3.833]
    desc_pos2 = [-321.222, 185.189, 335.520, -179.030, -1.284, -29.869]
    desc_pos3 = [-487.434, 154.362, 308.576, 176.600, 0.268, -14.061]
    desc_pos4 = [-443.165, 147.881, 480.951, 179.511, -0.775, -15.409]

    # Initialize other parameters
    offset_pos = [0, 0, 0, 0, 0, 0]
    epos = [0, 0, 0, 0]

    tool = 0
    user = 0
    vel = 100.0
    acc = 100.0
    ovl = 100.0
    blendT = 0.0
    blendR = 0.0
    flag = 0
    search = 0

    robot.SetSpeed(20)

    # Execute movements
    rtn = robot.MoveJ(joint_pos=j1, tool=tool, user=user, vel=vel, blendT=blendT)
    print(f"movej errcode: {rtn}")

    rtn = robot.MoveL(desc_pos=desc_pos2, tool=tool, user=user, vel=vel, blendR=blendR)
    print(f"movel errcode: {rtn}")

    rtn = robot.MoveC(desc_pos_p=desc_pos3, tool_p=tool, user_p=user, desc_pos_t=desc_pos4, tool_t=tool, user_t=user, blendR=blendR)
    print(f"movec errcode: {rtn}")

    rtn = robot.MoveJ(joint_pos=j2, tool=tool, user=user, vel=vel, blendT=blendT)
    print(f"movej errcode: {rtn}")

    rtn = robot.Circle(desc_pos_p=desc_pos3, tool_p=tool, user_p=user, desc_pos_t=desc_pos1, tool_t=tool, user_t=user)
    print(f"circle errcode: {rtn}")

    rtn = robot.MoveCart(desc_pos=desc_pos4, tool=tool, user=user, blendT=blendT)
    print(f"MoveCart errcode: {rtn}")

    robot.CloseRPC()


def TestSpiral(self):
    """螺旋运动测试函数"""
    # 定义点位数据（使用列表格式）
    joint_pos = [-11.904, -99.669, 117.473, -108.616, -91.726, 74.256]  # 关节角(度)
    desc_pos = [-419.524, -13.000, 351.569, -178.118, 0.314, 3.833]  # 笛卡尔位姿(x,y,z,rx,ry,rz)
    offset_pos1 = [50, 0, 0, -30, 0, 0]  # 初始偏移量
    offset_pos2 = [50, 0, 0, -5, 0, 0]  # 螺旋偏移量
    epos = [0, 0, 0, 0]  # 外部轴位置

    # 螺旋运动参数配置
    sp = [5,5.0,50.0,10.0,10.0,0]

    # 运动参数
    tool = 0  # 工具坐标系编号
    user = 0  # 用户坐标系编号
    vel = 100.0  # 速度百分比
    acc = 100.0  # 加速度百分比
    ovl = 100.0  # 速度缩放因子
    blendT = 0.0  # 过渡时间
    flag = 2  # 运动标志位

    robot.SetSpeed(20)  # 设置全局速度

    # 关节运动到起始点
    rtn = robot.MoveJ(joint_pos=joint_pos, tool=tool, user=user, exaxis_pos=epos, blendT=blendT, offset_flag=flag, offset_pos=offset_pos1)
    print(f"MoveJ 错误码: {rtn}")

    # 执行螺旋运动
    rtn = robot.NewSpiral(desc_pos=desc_pos, tool=tool, user=user, param=sp, exaxis_pos=epos, offset_flag=flag, offset_pos=offset_pos2)
    print(f"NewSpiral 错误码: {rtn}")
    robot.CloseRPC()


def TestServoJ(self):
    """伺服关节运动测试函数"""

    # 初始化位置和参数
    joint_pos = [0.0,0.0,0.0,0.0,0.0,0.0]  # 初始化6个关节角（单位：度）
    epos = [0.0,0.0,0.0,0.0]  # 外部轴位置

    # 运动参数配置
    vel = 0.0  # 速度（单位：度/秒）
    acc = 0.0  # 加速度（单位：度/秒²）
    cmdT = 0.008  # 命令周期（单位：秒）
    filterT = 0.0  # 滤波时间
    gain = 0.0  # 增益系数
    flag = 0  # 运动标志位
    count = 500  # 运动次数
    dt = 0.1  # 每次运动的增量（单位：度）

    # 获取当前关节位置
    ret, joint_pos = robot.GetActualJointPosDegree(0)
    if ret == 0:
        print("开始伺服关节运动...")
        robot.ServoMoveStart()

        # 执行伺服运动循环
        while count > 0:
            robot.ServoJ(joint_pos=joint_pos, axisPos=epos, cmdT=cmdT, filterT=filterT, gain=gain)
            joint_pos[0] += dt  # 仅第一个关节每次增加dt度
            count -= 1
            time.sleep(cmdT)  # 等待一个命令周期

        robot.ServoMoveEnd()
        print("伺服关节运动完成")


        robot.CloseRPC()


def TestServoCart(self):
    """笛卡尔空间伺服运动测试函数"""
    # 初始化笛卡尔位姿
    desc_pos_dt = [0.0,0.0,0.0,0.0,0.0,0.0]  # [x, y, z, rx, ry, rz]
    desc_pos_dt[2] = -0.5  # z轴偏移-0.5mm

    # 运动参数配置
    pos_gain = [0.0, 0.0, 1.0, 0.0, 0.0, 0.0]  # 位置增益（仅在z轴生效）
    mode = 2  # 运动模式
    vel = 0.0  # 速度
    acc = 0.0  # 加速度
    cmdT = 0.008  # 命令周期（秒）
    filterT = 0.0  # 滤波时间
    gain = 0.0  # 增益系数
    flag = 0  # 运动标志位
    count = 100  # 循环次数

    robot.SetSpeed(20)  # 设置全局速度

    while count > 0:
        robot.ServoCart(mode=mode, desc_pos=desc_pos_dt, pos_gain=pos_gain, acc=acc, vel=vel, cmdT=cmdT, filterT=filterT, gain=gain)
        count -= 1
        time.sleep(cmdT)  # 等待一个命令周期
    robot.CloseRPC()


def TestSpline(self):
    """样条运动测试函数"""
    # 定义关节角点位（单位：度）
    joint_points = [
        [-11.904, -99.669, 117.473, -108.616, -91.726, 74.256],  # j1
        [-45.615, -106.172, 124.296, -107.151, -91.282, 74.255],  # j2
        [-61.954, -84.409, 108.153, -116.316, -91.283, 74.260],  # j3
        [-89.575, -80.276, 102.713, -116.302, -91.284, 74.267]  # j4
    ]

    # 定义笛卡尔点位 [x,y,z,rx,ry,rz]（单位：mm和度）
    cart_points = [
        [-419.524, -13.000, 351.569, -178.118, 0.314, 3.833],  # desc_pos1
        [-321.222, 185.189, 335.520, -179.030, -1.284, -29.869],  # desc_pos2
        [-327.622, 402.230, 320.402, -178.067, 2.127, -46.207],  # desc_pos3
        [-104.066, 544.321, 327.023, -177.715, 3.371, -73.818]  # desc_pos4
    ]

    # 通用参数
    offset_pos = [0] * 6  # 位姿偏移量
    epos = [0] * 4  # 外部轴位置
    tool = user = 0  # 工具/用户坐标系编号
    vel = acc = ovl = 100.0  # 速度/加速度/速度缩放（%）
    blendT = -1.0  # 过渡时间（-1表示自动计算）
    flag = 0  # 运动标志位

    robot.SetSpeed(20)  # 设置全局速度（%）

    # 运动到起始点
    err1 = robot.MoveJ(joint_pos=joint_points[0],tool=tool, user=user,vel=vel)
    print(f"MoveJ 错误码: {err1}")

    # 执行样条运动
    robot.SplineStart()
    robot.SplinePTP(joint_pos=joint_points[0],tool=tool, user=user)
    robot.SplinePTP(joint_pos=joint_points[1],tool=tool, user=user)
    robot.SplinePTP(joint_pos=joint_points[2],tool=tool, user=user)
    robot.SplinePTP(joint_pos=joint_points[3],tool=tool, user=user)
    robot.SplineEnd()

    robot.CloseRPC()


def TestNewSpline(self):
    j1 = [-11.904, -99.669, 117.473, -108.616, -91.726, 74.256]
    j2 = [-45.615, -106.172, 124.296, -107.151, -91.282, 74.255]
    j3 = [-61.954, -84.409, 108.153, -116.316, -91.283, 74.260]
    j4 = [-89.575, -80.276, 102.713, -116.302, -91.284, 74.267]
    j5 = [-95.228, -54.621, 73.691, -112.245, -91.280, 74.268]

    desc_pos1 = [-419.524, -13.000, 351.569, -178.118, 0.314, 3.833]
    desc_pos2 = [-321.222, 185.189, 335.520, -179.030, -1.284, -29.869]
    desc_pos3 = [-327.622, 402.230, 320.402, -178.067, 2.127, -46.207]
    desc_pos4 = [-104.066, 544.321, 327.023, -177.715, 3.371, -73.818]
    desc_pos5 = [-33.421, 732.572, 275.103, -177.907, 2.709, -79.482]

    offset_pos = [0, 0, 0, 0, 0, 0]
    epos = [0, 0, 0, 0]

    tool = 0
    user = 0
    vel = 100.0
    acc = 100.0
    ovl = 100.0
    blendT = -1.0
    flag = 0

    robot.SetSpeed(20)

    err1 = robot.MoveJ(joint_pos=j1, tool=tool, user=user, vel=vel)
    print(f"movej errcode:{err1}")

    robot.NewSplineStart(1, 2000)
    robot.NewSplinePoint(desc_pos=desc_pos1, tool=tool, user=user, vel=vel, lastFlag=-1, blendR=0)
    robot.NewSplinePoint(desc_pos=desc_pos2, tool=tool, user=user, vel=vel, lastFlag=-1, blendR=0)
    robot.NewSplinePoint(desc_pos=desc_pos3, tool=tool, user=user, vel=vel, lastFlag=-1, blendR=0)
    robot.NewSplinePoint(desc_pos=desc_pos4, tool=tool, user=user, vel=vel, lastFlag=-1, blendR=0)
    robot.NewSplinePoint(desc_pos=desc_pos5, tool=tool, user=user, vel=vel, lastFlag=-1, blendR=0)
    robot.NewSplineEnd()

    robot.CloseRPC()

def TestPause(self):
    j1 =[-11.904, -99.669, 117.473, -108.616, -91.726, 74.256]
    j5 =[-95.228, -54.621, 73.691, -112.245, -91.280, 74.268]
    desc_pos1 = [-419.524, -13.000, 351.569, -178.118, 0.314, 3.833]
    desc_pos5 = [-33.421, 732.572, 275.103, -177.907, 2.709, -79.482]
    offset_pos = [0, 0, 0, 0, 0, 0]
    epos = [0, 0, 0, 0]

    tool = 0
    user = 0
    vel = 100.0
    acc = 100.0
    ovl = 100.0
    blendT = -1.0
    flag = 0

    robot.SetSpeed(20)

    rtn = robot.MoveJ(joint_pos=j1, tool=tool, user=user, vel=vel)
    rtn = robot.MoveJ(joint_pos=j5, tool=tool, user=user, vel=vel, blendT=1)
    time.sleep(1)
    robot.PauseMotion()

    time.sleep(1)
    robot.ResumeMotion()

    time.sleep(1)
    robot.StopMotion()

    time.sleep(1)

    robot.CloseRPC()


def TestOffset(self):

    j1 = [-11.904, -99.669, 117.473, -108.616, -91.726, 74.256]
    j2 = [-45.615, -106.172, 124.296, -107.151, -91.282, 74.255]

    desc_pos1 = [-419.524, -13.000, 351.569, -178.118, 0.314, 3.833]
    desc_pos2 = [-321.222, 185.189, 335.520, -179.030, -1.284, -29.869]

    offset_pos = [0, 0, 0, 0, 0, 0]
    offset_pos1 = [0, 0, 50, 0, 0, 0]
    epos = [0, 0, 0, 0]

    tool = 0
    user = 0
    vel = 100.0
    acc = 100.0
    ovl = 100.0
    blendT = -1.0
    flag = 0

    robot.SetSpeed(20)

    # First movement without offset
    robot.MoveJ(joint_pos=j1,tool=tool, user=user, vel=vel)
    robot.MoveJ(joint_pos=j2, tool=tool, user=user, vel=vel)
    time.sleep(1)

    # Second movement with Z-axis offset
    robot.PointsOffsetEnable(flag=0, offset_pos=offset_pos1)
    robot.MoveJ(joint_pos=j1,tool=tool, user=user, vel=vel)
    robot.MoveJ(joint_pos=j2, tool=tool, user=user, vel=vel)
    robot.PointsOffsetDisable()

    robot.CloseRPC()

def TestMoveAO(self):

    # Define joint positions
    j1 = [-11.904, -99.669, 117.473, -108.616, -91.726, 74.256]
    j2 = [-45.615, -106.172, 124.296, -107.151, -91.282, 74.255]

    # Define Cartesian positions
    desc_pos1 = [-419.524, -13.000, 351.569, -178.118, 0.314, 3.833]
    desc_pos2 = [-321.222, 185.189, 335.520, -179.030, -1.284, -29.869]

    # Define offsets
    offset_pos = [0, 0, 0, 0, 0, 0]
    offset_pos1 = [0, 0, 50, 0, 0, 0]
    epos = [0, 0, 0, 0]

    # Movement parameters
    tool = 0
    user = 0
    vel = 20.0
    acc = 20.0
    ovl = 100.0
    blendT = -1.0
    flag = 0

    robot.SetSpeed(20)

    # Base coordinate system AO movement
    robot.MoveAOStart(0, 100, 100, 20)
    robot.MoveJ(joint_pos=j1,tool=tool, user=user, vel=vel)
    robot.MoveJ(joint_pos=j2, tool=tool, user=user, vel=vel)
    robot.MoveAOStop()

    time.sleep(1)

    # Tool coordinate system AO movement
    robot.MoveToolAOStart(0, 100, 100, 20)
    robot.MoveJ(joint_pos=j1,tool=tool, user=user, vel=vel)
    robot.MoveJ(joint_pos=j2, tool=tool, user=user, vel=vel)
    robot.MoveToolAOStop()

    robot.CloseRPC()


def TestAODO(self):
    # 初始化参数
    status = 1  # 数字输出状态
    smooth = 0   # 平滑过渡
    block = 0    # 是否阻塞

    # 循环设置16个数字输出（DO）为高电平
    for i in range(16):
        robot.SetDO(i, status, smooth, block)
        time.sleep(0.3)  # 300ms延时

    status = 0  # 切换为低电平

    # 循环设置16个数字输出（DO）为低电平
    for i in range(16):
        robot.SetDO(i, status, smooth, block)
        time.sleep(0.3)

    status = 1  # 重新设为高电平

    # 设置2个工具数字输出（Tool DO）为高电平
    for i in range(2):
        robot.SetToolDO(i, status, smooth, block)
        time.sleep(1)  # 1秒延时

    status = 0  # 切换为低电平

    # 设置2个工具数字输出（Tool DO）为低电平
    for i in range(2):
        robot.SetToolDO(i, status, smooth, block)
        time.sleep(1)

    # 模拟输出（AO）渐变测试
    for i in range(100):
        robot.SetAO(0, i, block)  # 0号模拟输出
        time.sleep(0.03)  # 30ms延时

    # 工具模拟输出（Tool AO）渐变测试
    for i in range(100):
        robot.SetToolAO(0, i, block)  # 0号工具模拟输出
        time.sleep(0.03)

    # 关闭连接
    robot.CloseRPC()

import time

def TestGetDIAI(self):
    # 初始化参数
    block = 0  # 非阻塞模式

    # 获取数字输入(DI)状态
    error,di = robot.GetDI(0, block)
    print(f"di0: {di}")

    # 获取工具数字输入(Tool DI)状态
    error,tool_di = robot.GetToolDI(1, block)
    print(f"tool_di1: {tool_di}")

    # 获取模拟输入(AI)值
    error,ai = robot.GetAI(0, block)
    print(f"ai0: {ai:.2f}")  # 保留两位小数

    # 获取工具模拟输入(Tool AI)值
    error,tool_ai = robot.GetToolAI(0, block)
    print(f"tool_ai0: {tool_ai:.2f}")

    # 获取轴点记录按钮状态
    error,button_state = robot.GetAxlePointRecordBtnState()
    print(f"_button_state is: {button_state}")

    # 获取工具数字输出(Tool DO)状态
    error,tool_do_state = robot.GetToolDO()
    print(f"tool DO state: {tool_do_state}")

    # 获取数字输出(DO)状态(高低字节)
    error,[do_state_h, do_state_l] = robot.GetDO()
    print(f"DO state hight  : {do_state_h}")
    print(f"DO state low : {do_state_l}")

    # 关闭连接
    robot.CloseRPC()

def TestWaitDIAI(self):
    # 等待数字输入DI0变为1，超时1000ms，阻塞模式
    rtn = robot.WaitDI(0, 1, 1000, 1)
    print(f"WaitDI over; rtn is: {rtn}")

    # 等待多个数字输入(DI1和DI3)同时为1，超时1000ms
    rtn = robot.WaitMultiDI(1, 3, 3, 1000, 1)
    print(f"WaitDI over; rtn is: {rtn}")

    # 等待工具数字输入ToolDI1变为1，超时1000ms
    rtn = robot.WaitToolDI(1, 1, 1000, 1)
    print(f"WaitDI over; rtn is: {rtn}")

    # 等待模拟输入AI0值在0-50范围内，超时1000ms
    rtn = robot.WaitAI(0, 0, 50, 1000, 1)
    print(f"WaitDI over; rtn is: {rtn}")

    # 等待工具模拟输入ToolAI0值在0-50范围内，超时1000ms
    rtn = robot.WaitToolAI(0, 0, 50, 1000, 1)
    print(f"WaitDI over; rtn is: {rtn}")

    robot.CloseRPC()


def TestDOReset(self):

    # 循环设置16个数字输出(DO)为高电平，间隔300ms
    for i in range(16):
        robot.SetDO(i, 1, 0, 0)  # 设置DO为1，无平滑，非阻塞
        time.sleep(0.3)  # 300ms延时

    # 重置各类输出信号
    resetFlag = 1
    robot.SetOutputResetCtlBoxDO(resetFlag)    # 重置控制箱DO
    robot.SetOutputResetCtlBoxAO(resetFlag)    # 重置控制箱AO
    robot.SetOutputResetAxleDO(resetFlag)      # 重置轴DO
    robot.SetOutputResetAxleAO(resetFlag)      # 重置轴AO
    robot.SetOutputResetExtDO(resetFlag)       # 重置扩展DO
    robot.SetOutputResetExtAO(resetFlag)       # 重置扩展AO
    robot.SetOutputResetSmartToolDO(resetFlag) # 重置智能工具DO

    # 加载并运行Lua程序
    robot.ProgramLoad("/fruser/test0610.lua")
    robot.ProgramRun()

    # 关闭连接
    robot.CloseRPC()


def TestTCPCompute(self):
    # 定义6个笛卡尔坐标点
    p1Desc = [186.331, 487.913, 209.850, 149.030, 0.688, -114.347]
    p2Desc = [69.721, 535.073, 202.882, -144.406, -14.775, -89.012]
    p3Desc = [146.861, 578.426, 205.598, 175.997, -36.178, -93.437]
    p4Desc = [136.284, 509.876, 225.613, 178.987, 1.372, -100.696]
    p5Desc = [138.395, 505.972, 298.016, 179.134, 2.147, -101.110]
    p6Desc = [105.553, 454.325, 232.017, -179.426, 0.444, -99.952]

    # 定义6个关节位置
    p1Joint = [-127.876, -75.341, 115.417, -122.741, -59.820, 74.300]
    p2Joint = [-101.780, -69.828, 110.917, -125.740, -127.841, 74.300]
    p3Joint = [-112.851, -60.191, 86.566, -80.676, -97.463, 74.300]
    p4Joint = [-116.397, -76.281, 113.845, -128.611, -88.654, 74.299]
    p5Joint = [-116.814, -82.333, 109.162, -118.662, -88.585, 74.302]
    p6Joint = [-115.649, -84.367, 122.447, -128.663, -90.432, 74.303]

    exaxisPos = [0, 0, 0, 0]  # 外部轴位置
    offdese = [0, 0, 0, 0, 0, 0]  # 偏移量

    posJ = [p1Joint, p2Joint, p3Joint, p4Joint, p5Joint, p6Joint]  # 关节位置数组
    # coordRtn = DescPose()  # 用于存储计算结果的坐标

    # 使用6个点计算工具坐标系
    rtn,coordRtn = robot.ComputeToolCoordWithPoints(1, posJ)
    print(
        f"ComputeToolCoordWithPoints    {rtn}  coord is {coordRtn[0]} {coordRtn[1]} {coordRtn[2]} {coordRtn[3]} {coordRtn[4]} {coordRtn[5]}")

    # 移动到各点并设置工具点
    robot.MoveJ(joint_pos=p1Joint,tool=0, user=0, vel=100)
    robot.SetToolPoint(1)
    robot.MoveJ(joint_pos=p2Joint,tool=0, user=0, vel=100)
    robot.SetToolPoint(2)
    robot.MoveJ(joint_pos=p3Joint,tool=0, user=0, vel=100)
    robot.SetToolPoint(3)
    robot.MoveJ(joint_pos=p4Joint,tool=0, user=0, vel=100)
    robot.SetToolPoint(4)
    robot.MoveJ(joint_pos=p5Joint,tool=0, user=0, vel=100)
    robot.SetToolPoint(5)
    robot.MoveJ(joint_pos=p6Joint,tool=0, user=0, vel=100)
    robot.SetToolPoint(6)

    # 计算工具坐标系
    rtn,coordRtn = robot.ComputeTool()
    print(
        f"6 Point ComputeTool        {rtn}  coord is {coordRtn[0]} {coordRtn[1]} {coordRtn[2]} {coordRtn[3]} {coordRtn[4]} {coordRtn[5]}")
    robot.SetToolList(1, coordRtn, 0, 0, 0)

    # 使用4点法计算TCP
    robot.MoveJ(joint_pos=p1Joint,tool=0, user=0, vel=100)
    robot.SetTcp4RefPoint(1)
    robot.MoveJ(joint_pos=p2Joint,tool=0, user=0, vel=100)
    robot.SetTcp4RefPoint(2)
    robot.MoveJ(joint_pos=p3Joint,tool=0, user=0, vel=100)
    robot.SetTcp4RefPoint(3)
    robot.MoveJ(joint_pos=p4Joint,tool=0, user=0, vel=100)
    robot.SetTcp4RefPoint(4)

    rtn,coordRtn = robot.ComputeTcp4()
    print(
        f"4 Point ComputeTool        {rtn}  coord is {coordRtn[0]} {coordRtn[1]} {coordRtn[2]} {coordRtn[3]} {coordRtn[4]} {coordRtn[5]}")

    robot.SetToolCoord(2, coordRtn, 0, 0, 1, 0)  # 设置工具坐标系

    # 获取TCP偏移量
    # getCoord = DescPose()
    rtn,getCoord = robot.GetTCPOffset(0)
    print(
        f"GetTCPOffset    {rtn}  coord is {getCoord[0]} {getCoord[1]} {getCoord[2]} {getCoord[3]} {getCoord[4]} {getCoord[5]}")

    robot.CloseRPC()  # 关闭连接


def TestWobjCoord(self):
    # 定义3个笛卡尔坐标点(使用列表表示)
    p1Desc = [-89.606, 779.517, 193.516, 178.000, 0.476, -92.484]
    p2Desc = [-24.656, 850.384, 191.361, 177.079, -2.058, -95.355]
    p3Desc = [-99.813, 766.661, 241.878, -176.817, 1.917, -91.604]

    # 定义3个关节位置(使用列表表示)
    p1Joint = [-108.145, -50.137, 85.818, -125.599, -87.946, 74.329]
    p2Joint = [-111.024, -41.538, 69.222, -114.913, -87.743, 74.329]
    p3Joint = [-107.266, -56.116, 85.971, -122.560, -92.548, 74.331]

    exaxisPos = [0, 0, 0, 0]  # 外部轴位置
    offdese = [0, 0, 0, 0, 0, 0]  # 偏移量

    posTCP = [p1Desc, p2Desc, p3Desc]  # TCP位置数组
    # coordRtn = DescPose()  # 用于存储计算结果的坐标

    # 使用3个点计算工件坐标系
    rtn,coordRtn = robot.ComputeWObjCoordWithPoints(1, posTCP, 0)
    print(
        f"ComputeWObjCoordWithPoints    {rtn}  coord is {coordRtn[0]} {coordRtn[1]} {coordRtn[2]} {coordRtn[3]} {coordRtn[4]} {coordRtn[5]}")

    # 移动到各点并设置工件坐标系点
    robot.MoveJ(joint_pos=p1Joint,tool=1, user=0, vel=100)
    robot.SetWObjCoordPoint(1)
    robot.MoveJ(joint_pos=p2Joint,tool=1, user=0, vel=100)
    robot.SetWObjCoordPoint(2)
    robot.MoveJ(joint_pos=p3Joint,tool=1, user=0, vel=100)
    robot.SetWObjCoordPoint(3)

    # 计算工件坐标系
    rtn,coordRtn = robot.ComputeWObjCoord(1, 0)
    print(
        f"ComputeWObjCoord   {rtn}  coord is {coordRtn[0]} {coordRtn[1]} {coordRtn[2]} {coordRtn[3]} {coordRtn[4]} {coordRtn[5]}")

    # 设置工件坐标系
    robot.SetWObjCoord(1, coordRtn, 0)
    robot.SetWObjList(1, coordRtn, 0)

    # 获取工件坐标系偏移量
    # getWobjDesc = DescPose()
    rtn,getWobjDesc = robot.GetWObjOffset(0)
    print(
        f"GetWObjOffset    {rtn}  coord is {getWobjDesc[0]} {getWobjDesc[1]} {getWobjDesc[2]} {getWobjDesc[3]} {getWobjDesc[4]} {getWobjDesc[5]}")

    robot.CloseRPC()  # 关闭连接

def TestExtCoord(self):
    p1Desc = [-89.606, 779.517, 193.516, 178.000, 0.476, -92.484]
    p1Joint = [-108.145, -50.137, 85.818, -125.599, -87.946, 74.329]

    p2Desc = [-24.656, 850.384, 191.361, 177.079, -2.058, -95.355]
    p2Joint = [-111.024, -41.538, 69.222, -114.913, -87.743, 74.329]

    p3Desc = [-99.813, 766.661, 241.878, -176.817, 1.917, -91.604]
    p3Joint = [-107.266, -56.116, 85.971, -122.560, -92.548, 74.331]

    exaxisPos = [0, 0, 0, 0]
    offdese = [0, 0, 0, 0, 0, 0]

    posTCP = [p1Desc, p2Desc, p3Desc]
    # coordRtn = DescPose()

    robot.MoveJ(joint_pos=p1Joint,tool=1, user=0, vel=50)
    robot.SetExTCPPoint(1)
    robot.MoveJ(joint_pos=p2Joint,tool=1, user=0, vel=50)
    robot.SetExTCPPoint(2)
    robot.MoveJ(joint_pos=p3Joint,tool=1, user=0, vel=50)
    robot.SetExTCPPoint(3)
    rtn,coordRtn = robot.ComputeExTCF()
    print(f"ComputeExTCF {rtn}  coord is {coordRtn[0]} {coordRtn[1]} {coordRtn[2]} {coordRtn[3]} {coordRtn[4]} {coordRtn[5]}")

    robot.SetExToolCoord(1, coordRtn, offdese)
    robot.SetExToolList(1, coordRtn, offdese)

    robot.CloseRPC()

def TestLoadInstall(self):

    for i in range(1, 100):
        robot.SetSpeed(i)
        robot.SetOaccScale(i)
        time.sleep(0.03)  # 30ms in seconds

    error,defaultVel = robot.GetDefaultTransVel()
    print(f"GetDefaultTransVel is {defaultVel}")

    for i in range(1, 21):
        robot.SetSysVarValue(i, i + 0.5)
        time.sleep(0.1)  # 100ms in seconds

    for i in range(1, 21):
        value = robot.GetSysVarValue(i)
        print(f"sys value {i} is: {value}")
        time.sleep(0.1)  # 100ms in seconds

    robot.SetLoadWeight(0, 2.5)

    robot.SetLoadCoord(3.0,4.0,5.0)

    time.sleep(1)  # 1000ms in seconds

    error,getLoad = robot.GetTargetPayload(0)

    error,getLoadTran = robot.GetTargetPayloadCog(0)
    print(f"get load is {getLoad}; get load cog is {getLoadTran[0]} {getLoadTran[1]} {getLoadTran[2]}")

    robot.SetRobotInstallPos(0)
    robot.SetRobotInstallAngle(15.0, 25.0)

    error,[anglex, angley] = robot.GetRobotInstallAngle()
    print(f"GetRobotInstallAngle x: {anglex}; y: {angley}")

    robot.CloseRPC()

def TestFriction(self):
    lcoeff = [0.9, 0.9, 0.9, 0.9, 0.9, 0.9]
    wcoeff = [0.4, 0.4, 0.4, 0.4, 0.4, 0.4]
    ccoeff = [0.6, 0.6, 0.6, 0.6, 0.6, 0.6]
    fcoeff = [0.5, 0.5, 0.5, 0.5, 0.5, 0.5]

    rtn = robot.FrictionCompensationOnOff(1)
    print(f"FrictionCompensationOnOff rtn is {rtn}")

    rtn = robot.SetFrictionValue_level(lcoeff)
    print(f"SetFrictionValue_level rtn is {rtn}")

    rtn = robot.SetFrictionValue_wall(wcoeff)
    print(f"SetFrictionValue_wall rtn is {rtn}")

    rtn = robot.SetFrictionValue_ceiling(ccoeff)
    print(f"SetFrictionValue_ceiling rtn is {rtn}")

    rtn = robot.SetFrictionValue_freedom(fcoeff)
    print(f"SetFrictionValue_freedom rtn is {rtn}")

    robot.CloseRPC()

def TestGetError(self):
    # In Python, we typically return values rather than using output parameters
    p1Joint = [-108.145, -50.137, 85.818, -125.599, -87.946, 74.329]
    robot.MoveJ(joint_pos=p1Joint, tool=5, user=2, vel=50)
    time.sleep(1)
    error,[maincode, subcode] = robot.GetRobotErrorCode()
    print(f"robot maincode is {maincode}; subcode is {subcode}")
    time.sleep(1)

    robot.ResetAllError()

    time.sleep(1)  # 1000ms in seconds

    error,[maincode, subcode] = robot.GetRobotErrorCode()
    print(f"robot maincode is {maincode}; subcode is {subcode}")

    robot.CloseRPC()


def TestCollision(self):
    # Collision detection configuration
    mode = 0
    config = 1
    level1 = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
    level2 = [50.0, 20.0, 30.0, 40.0, 50.0, 60.0]

    rtn = robot.SetAnticollision(mode, level1, config)
    print(f"SetAnticollision mode 0 rtn is {rtn}")

    mode = 1
    rtn = robot.SetAnticollision(mode, level2, config)
    print(f"SetAnticollision mode 1 rtn is {rtn}")

    # Define joint positions and poses
    p1Joint = [-11.904, -99.669, 117.473, -108.616, -91.726, 74.256]
    p2Joint = [-45.615, -106.172, 124.296, -107.151, -91.282, 74.255]

    p1Desc = [-419.524, -13.000, 351.569, -178.118, 0.314, 3.833]
    p2Desc = [-321.222, 185.189, 335.520, -179.030, -1.284, -29.869]

    exaxisPos = [0.0, 0.0, 0.0, 0.0]
    offdese = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    # Execute movement and collision detection
    robot.MoveL(desc_pos=p2Desc, tool=0, user=0, vel=100, blendR=2)
    robot.ResetAllError()

    safety = [5, 5, 5, 5, 5, 5]
    rtn = robot.SetCollisionStrategy(3, 1000, 150, 250, safety)
    print(f"SetCollisionStrategy rtn is {rtn}")

    jointDetectionThreshould = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
    tcpDetectionThreshould = [60, 60, 60, 60, 60, 60]
    rtn = robot.CustomCollisionDetectionStart(3, jointDetectionThreshould, tcpDetectionThreshould, 0)
    print(f"CustomCollisionDetectionStart rtn is {rtn}")

    # Perform movements with collision detection
    robot.MoveL(desc_pos=p1Desc, tool=0, user=0, vel=100)
    robot.MoveL(desc_pos=p2Desc, tool=0, user=0, vel=100)

    rtn = robot.CustomCollisionDetectionEnd()
    print(f"CustomCollisionDetectionEnd rtn is {rtn}")

    robot.CloseRPC()


def TestLimit(self):
    # Set joint limits
    plimit = [170.0, 80.0, 150.0, 80.0, 170.0, 160.0]
    robot.SetLimitPositive(plimit)

    nlimit = [-170.0, -260.0, -150.0, -260.0, -170.0, -160.0]
    robot.SetLimitNegative(nlimit)

    # Get and print soft limits
    error,neg_deg = robot.GetJointSoftLimitDeg(0)
    print(f"pos limit deg: {neg_deg[1]}, {neg_deg[3]}, {neg_deg[5]}, {neg_deg[7]}, {neg_deg[9]}, {neg_deg[11]}")
    print(f"neg limit deg: {neg_deg[0]}, {neg_deg[2]}, {neg_deg[4]}, {neg_deg[6]}, {neg_deg[8]}, {neg_deg[10]}")

    robot.CloseRPC()


def TestCollisionMethod(self):
    # Configure collision detection
    rtn = robot.SetCollisionDetectionMethod(0,0)

    # Test static collision detection
    rtn = robot.SetStaticCollisionOnOff(1)
    print(f"SetStaticCollisionOnOff On rtn is {rtn}")

    time.sleep(5)  # Wait for 5 seconds (5000ms)

    rtn = robot.SetStaticCollisionOnOff(0)
    print(f"SetStaticCollisionOnOff Off rtn is {rtn}")

    robot.CloseRPC()


def TestPowerLimit(self):
    # Enable drag teaching and set power limit
    robot.DragTeachSwitch(1)
    robot.SetPowerLimit(1, 200)

    # Initialize torque array
    # torques = [0.0,0.0,0.0,0.0,0.0,0.0]  # Create list of 6 zeros
    error,torques = robot.GetJointTorques(1)

    # ServoJT control loop
    count = 100
    robot.ServoJTStart()  # Start servoJT mode

    while count > 0:
        error = robot.ServoJT(torques, 0.001)
        count -= 1
        time.sleep(0.001)  # 1ms delay

    error = robot.ServoJTEnd()

    robot.DragTeachSwitch(0)
    robot.CloseRPC()

def TestServoJT(self):
    # Enable drag teaching
    robot.DragTeachSwitch(1)

    # Get current joint torques
    # torques = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Torque array for 6 joints
    error,torques = robot.GetJointTorques(1)

    # Start servoJT mode
    robot.ServoJTStart()

    # Execute servo control loop
    count = 100
    while count > 0:
        error = robot.ServoJT(torques, 0.001)
        count -= 1
        time.sleep(0.001)  # 1ms delay

    # End servoJT mode
    error = robot.ServoJTEnd()

    # Disable drag teaching
    robot.DragTeachSwitch(0)

    # Close connection
    robot.CloseRPC()

def TestGetStatus(self):
    # Get robot installation angles
    error,[yangle, zangle] = robot.GetRobotInstallAngle()
    print(f"yangle:{yangle},zangle:{zangle}")

    # Get joint positions
    # j_deg = JointPos()
    error,j_deg = robot.GetActualJointPosDegree(0)
    print(f"joint pos deg:{j_deg[0]},{j_deg[1]},{j_deg[2]},{j_deg[3]},{j_deg[4]},{j_deg[5]}")

    # Get joint speeds
    # jointSpeed = [0.0] * 6
    error,jointSpeed = robot.GetActualJointSpeedsDegree(0)
    print(f"joint speeds deg:{jointSpeed[0]},{jointSpeed[1]},{jointSpeed[2]},{jointSpeed[3]},{jointSpeed[4]},{jointSpeed[5]}")

    # Get joint accelerations
    # jointAcc = [0.0] * 6
    error,jointAcc = robot.GetActualJointAccDegree(0)
    print(f"joint acc deg:{jointAcc[0]},{jointAcc[1]},{jointAcc[2]},{jointAcc[3]},{jointAcc[4]},{jointAcc[5]}")

    # Get TCP speeds
    error,[tcp_speed, ori_speed] = robot.GetTargetTCPCompositeSpeed(0)
    print(f"GetTargetTCPCompositeSpeed tcp {tcp_speed}; ori {ori_speed}")

    error,[tcp_speed, ori_speed] = robot.GetActualTCPCompositeSpeed(0)
    print(f"GetActualTCPCompositeSpeed tcp {tcp_speed}; ori {ori_speed}")

    # targetSpeed = [0.0] * 6
    error,targetSpeed = robot.GetTargetTCPSpeed(0)
    print(f"GetTargetTCPSpeed {targetSpeed[0]},{targetSpeed[1]},{targetSpeed[2]},{targetSpeed[3]},{targetSpeed[4]},{targetSpeed[5]}")

    # actualSpeed = [0.0] * 6
    error,actualSpeed = robot.GetActualTCPSpeed(0)
    print(f"GetActualTCPSpeed {actualSpeed[0]},{actualSpeed[1]},{actualSpeed[2]},{actualSpeed[3]},{actualSpeed[4]},{actualSpeed[5]}")

    # Get TCP and flange poses
    # tcp = DescPose()
    error,tcp = robot.GetActualTCPPose(0)
    print(f"tcp pose:{tcp[0]},{tcp[1]},{tcp[2]},{tcp[3]},{tcp[4]},{tcp[5]}")

    # flange = DescPose()
    error,flange = robot.GetActualToolFlangePose(0)
    print(f"flange pose:{flange[0]},{flange[1]},{flange[2]},{flange[3]},{flange[4]},{flange[5]}")

    # Get TCP and work object numbers
    error,id = robot.GetActualTCPNum(0)
    print(f"tcp num:{id}")

    error,id = robot.GetActualWObjNum(0)
    print(f"wobj num:{id}")

    # Get joint torques
    # jtorque = [0.0] * 6
    error,jtorque = robot.GetJointTorques(0)
    print(f"torques:{jtorque[0]},{jtorque[1]},{jtorque[2]},{jtorque[3]},{jtorque[4]},{jtorque[5]}")

    # Get system clock
    error,t_ms = robot.GetSystemClock()
    print(f"system clock:{t_ms}")

    # Get configuration and status
    error,config = robot.GetRobotCurJointsConfig()
    print(f"joint config:{config}")

    error,motionDone = robot.GetRobotMotionDone()
    print(f"GetRobotMotionDone:{motionDone}")

    error,len = robot.GetMotionQueueLength()
    print(f"GetMotionQueueLength:{len}")

    error,emergState = robot.GetRobotEmergencyStopState()
    print(f"GetRobotEmergencyStopState:{emergState}")

    error,comstate = robot.GetSDKComState()
    print(f"GetSDKComState:{comstate}")

    # Get safety stop state
    error,[si0_state, si1_state] = robot.GetSafetyStopState()
    print(f"GetSafetyStopState:{si0_state} {si1_state}")

    # Get temperatures and torques
    # temp = [0.0] * 6
    error,temp = robot.GetJointDriverTemperature()
    print(f"Temperature:{temp[0]},{temp[1]},{temp[2]},{temp[3]},{temp[4]},{temp[5]}")

    # torque = [0.0] * 6
    error,torque = robot.GetJointDriverTorque()
    print(f"torque:{torque[0]},{torque[1]},{torque[2]},{torque[3]},{torque[4]},{torque[5]}")

    # Get real-time state
    # pkg = {}
    error,pkg = robot.GetRobotRealTimeState()

    robot.CloseRPC()


def TestInverseKin(self):
    # 创建关节位置对象
    j1 = [-11.904, -99.669, 117.473, -108.616, -91.726, 74.256]
    # 创建描述位姿对象
    desc_pos1 = [-419.524, -13.000, 351.569, -178.118, 0.314, 3.833]

    # 调用逆解函数
    error, inverseRtn = robot.GetInverseKin(0, desc_pos=desc_pos1, config=-1)
    print(f"dcs1 GetInverseKin rtn is {inverseRtn[0]}, {inverseRtn[1]}, {inverseRtn[2]}, "
          f"{inverseRtn[3]}, {inverseRtn[4]}, {inverseRtn[5]}")

    # 调用带参考的逆解函数
    error, inverseRtn = robot.GetInverseKinRef(0, desc_pos=desc_pos1, joint_pos_ref=j1)
    print(f"dcs1 GetInverseKinRef rtn is {inverseRtn[0]}, {inverseRtn[1]}, {inverseRtn[2]}, "
          f"{inverseRtn[3]}, {inverseRtn[4]}, {inverseRtn[5]}")

    # 检查是否有解
    error, hasResult = robot.GetInverseKinHasSolution(0, desc_pos=desc_pos1, joint_pos_ref=j1)
    print(f"dcs1 GetInverseKinRef result {hasResult}")

    # 正运动学计算
    error, forwordResult = robot.GetForwardKin(j1)
    print(f"jpos1 forwordResult rtn is {forwordResult[0]}, {forwordResult[1]}, {forwordResult[2]}, "
          f"{forwordResult[3]}, {forwordResult[4]}, {forwordResult[5]}")

    robot.CloseRPC()


def TestTPD(self):
    type = 1
    name = "tpd2025"
    period_ms = 4
    di_choose = 0
    do_choose = 0

    robot.SetTPDParam(name, period_ms)

    robot.Mode(1)
    time.sleep(1)  # 原robot.Sleep(1000)
    robot.DragTeachSwitch(1)
    robot.SetTPDStart(name, period_ms)
    print("SetTPDStart")
    time.sleep(10)  # 原robot.Sleep(10000)
    robot.SetWebTPDStop()
    robot.DragTeachSwitch(0)

    ovl = 100.0
    blend = 0

    rtn = robot.LoadTPD(name)
    print(f"LoadTPD rtn is: {rtn}")

    # start_pose = [0.0]*6
    error,start_pose = robot.GetTPDStartPose(name)
    print(f"start pose, xyz is: {start_pose[0]},{start_pose[1]},{start_pose[2]}. "
          f"rpy is: {start_pose[3]},{start_pose[4]},{start_pose[5]}")

    robot.MoveCart(start_pose, 0, 0, 100, 100)
    time.sleep(1)  # 原robot.Sleep(1000)

    rtn = robot.MoveTPD(name, blend, ovl)
    print(f"MoveTPD rtn is: {rtn}")
    time.sleep(5)

    robot.SetTPDDelete(name)
    robot.CloseRPC()


def TestGetTeachPoint(self):
    name = "P1"
    rtn, data = robot.GetRobotTeachingPoint(name)
    print(f"{rtn} name is: {name}")
    for i in range(20):
        print(f"data is: {data[i]}")

    rtn,que_len = robot.GetMotionQueueLength()
    print(f"GetMotionQueueLength rtn is: {rtn}, queue length is: {que_len}")

    retval,dh = robot.GetDHCompensation()
    print(f"retval is: {retval}")
    print(f"dh is: {dh[0]} {dh[1]} {dh[2]} {dh[3]} {dh[4]} {dh[5]}")

    error,sn = robot.GetRobotSN()
    print(f"robot SN is {sn[0]}")

    robot.CloseRPC()


def TestTraj(self):
    rtn = robot.TrajectoryJUpLoad("D://zUP/traj.txt")
    print(f"Upload TrajectoryJ A {rtn}")

    traj_file_name = "/fruser/traj/traj.txt"
    rtn = robot.LoadTrajectoryJ(traj_file_name, 100, 1)
    print(f"LoadTrajectoryJ {traj_file_name}, rtn is: {rtn}")

    rtn,traj_start_pose = robot.GetTrajectoryStartPose(traj_file_name)
    print(f"GetTrajectoryStartPose is: {rtn}")
    print(f"desc_pos:{traj_start_pose[0]},{traj_start_pose[1]},{traj_start_pose[2]},"
          f"{traj_start_pose[3]},{traj_start_pose[4]},{traj_start_pose[5]}")

    time.sleep(1)

    robot.SetSpeed(50)
    robot.MoveCart(traj_start_pose, 0, 0, 50, 100, 100)

    rtn,traj_num = robot.GetTrajectoryPointNum()
    print(f"GetTrajectoryStartPose rtn is: {rtn}, traj num is: {traj_num}")

    rtn = robot.SetTrajectoryJSpeed(50.0)
    print(f"SetTrajectoryJSpeed is: {rtn}")

    traj_force = [0.0,0.0,0.0,0.0,0.0,0.0]
    traj_force[0] = 10  # fx = 10
    rtn = robot.SetTrajectoryJForceTorque(traj_force)
    print(f"SetTrajectoryJForceTorque rtn is: {rtn}")

    rtn = robot.SetTrajectoryJForceFx(10.0)
    print(f"SetTrajectoryJForceFx rtn is: {rtn}")

    rtn = robot.SetTrajectoryJForceFy(0.0)
    print(f"SetTrajectoryJForceFy rtn is: {rtn}")

    rtn = robot.SetTrajectoryJForceFz(0.0)
    print(f"SetTrajectoryJForceFz rtn is: {rtn}")

    rtn = robot.SetTrajectoryJTorqueTx(10.0)
    print(f"SetTrajectoryJTorqueTx rtn is: {rtn}")

    rtn = robot.SetTrajectoryJTorqueTy(10.0)
    print(f"SetTrajectoryJTorqueTy rtn is: {rtn}")

    rtn = robot.SetTrajectoryJTorqueTz(10.0)
    print(f"SetTrajectoryJTorqueTz rtn is: {rtn}")

    rtn = robot.MoveTrajectoryJ()
    print(f"MoveTrajectoryJ rtn is: {rtn}")

    robot.CloseRPC()

def testtpd(self):
    rtn = robot.TrajectoryJUpLoad("D://zUP/testA.txt")
    print(f"Upload TrajectoryJ A {rtn}")

    rtn = robot.TrajectoryJUpLoad("D://zUP/testB.txt")
    print(f"Upload TrajectoryJ B {rtn}")

    rtn = robot.TrajectoryJDelete("testA.txt")
    print(f"Delete TrajectoryJ A {rtn}")

    rtn = robot.TrajectoryJDelete("testB.txt")
    print(f"Delete TrajectoryJ B {rtn}")

def TestLuaOp(self):
    program_name = "/fruser/test0610.lua"
    loaded_name = ""
    state = 0
    line = 0

    robot.Mode(0)
    robot.LoadDefaultProgConfig(0, program_name)
    robot.ProgramLoad(program_name)
    robot.ProgramRun()
    time.sleep(1)
    robot.ProgramPause()
    error,state = robot.GetProgramState()
    print(f"program state:{state}")
    error,line = robot.GetCurrentLine()
    print(f"current line:{line}")
    error,loaded_name = robot.GetLoadedProgram()
    print(f"program name:{loaded_name}")
    time.sleep(1)
    robot.ProgramResume()
    time.sleep(1)
    robot.ProgramStop()
    time.sleep(1)

    robot.CloseRPC()

import time

def TestLUAUpDownLoad(self):
    # 获取lua名称
    # luaNames = []
    rtn,lua_num,luaNames = robot.GetLuaList()
    print(f"res is:{rtn}")
    print(f"size is:{lua_num}")
    for name in luaNames:
        print(name)

    # 下载lua
    rtn = robot.LuaDownLoad("test0610.lua", "D://zDOWN/")
    print(f"LuaDownLoad rtn is:{rtn}")

    # 上传lua
    rtn = robot.LuaUpload("D://zDOWN/test0610.lua")
    print(f"LuaUpload rtn is:{rtn}")

    # 删除lua
    rtn = robot.LuaDelete("test0610.lua")
    print(f"LuaDelete rtn is:{rtn}")

    robot.CloseRPC()


def TestGripper(self):
    company = 4
    device = 0
    softversion = 0
    bus = 2
    index = 2
    act = 0
    max_time = 30000
    block = 0
    status = 0
    fault = 0
    active_status = 0
    current_pos = 0
    current = 0
    voltage = 0
    temp = 0
    speed = 0

    robot.SetGripperConfig(company, device, softversion, bus)
    time.sleep(1)
    error,[company, device, softversion, bus] = robot.GetGripperConfig()
    print(f"gripper config:{company},{device},{softversion},{bus}")

    robot.ActGripper(index, act)
    time.sleep(1)
    act = 1
    robot.ActGripper(index, act)
    time.sleep(1)

    error = robot.MoveGripper(index, 90, 50, 50, max_time, block, 0, 0, 0, 0)
    print(f"MoveGripper retval is:{error}")
    time.sleep(1)
    error = robot.MoveGripper(index, 30, 50, 0, max_time, block, 0, 0, 0, 0)
    print(f"MoveGripper retval is:{error}")

    error, [fault, status] = robot.GetGripperMotionDone()
    print(f"motion status:{fault},{status}")

    error, [fault, active_status] = robot.GetGripperActivateStatus()
    print(f"gripper active fault is:{fault},status is:{active_status}")

    error, [fault, current_pos] = robot.GetGripperCurPosition()
    print(f"fault is:{fault},current position is:{current_pos}")

    error, [fault, current] = robot.GetGripperCurCurrent()
    print(f"fault is:{fault},current current is:{current}")

    error, [fault, voltage] = robot.GetGripperVoltage()
    print(f"fault is:{fault},current voltage is:{voltage}")

    error, [fault, temp] = robot.GetGripperTemp()
    print(f"fault is:{fault},current temperature is:{temp}")

    error, [fault, speed] = robot.GetGripperCurSpeed()
    print(f"fault is:{fault},current speed is:{speed}")

    retval = 0
    prepick_pose = [0.0]*6
    postpick_pose = [0.0]*6

    p1Desc = [-419.524, -13.000, 351.569, -178.118, 0.314, 3.833]
    p2Desc = [-321.222, 185.189, 335.520, -179.030, -1.284, -29.869]

    retval, prepick_pose = robot.ComputePrePick(p1Desc, 10, 0)
    print(f"ComputePrePick retval is:{retval}")
    print(f"xyz is:{prepick_pose[0]},{prepick_pose[1]},{prepick_pose[2]};rpy is:{prepick_pose[3]},{prepick_pose[4]},{prepick_pose[5]}")

    retval, postpick_pose = robot.ComputePostPick(p2Desc, -10, 0)
    print(f"ComputePostPick retval is:{retval}")
    print(f"xyz is:{postpick_pose[0]},{postpick_pose[1]},{postpick_pose[2]};rpy is:{postpick_pose[3]},{postpick_pose[4]},{postpick_pose[5]}")

    robot.CloseRPC()


def TestRotGripperState(self):
    fault = 0
    rotNum = 0.0
    rotSpeed = 0
    rotTorque = 0

    error,fault, rotNum = robot.GetGripperRotNum()
    error,fault, rotSpeed = robot.GetGripperRotSpeed()
    error,fault, rotTorque = robot.GetGripperRotTorque()

    print(f"gripper rot num:{rotNum},gripper rotSpeed:{rotSpeed},gripper rotTorque:{rotTorque}")

    robot.CloseRPC()
    return 0


def TestConveyor(self):
    retval = robot.ConveyorStartEnd(1)
    print(f"ConveyorStartEnd retval is:{retval}")

    retval = robot.ConveyorPointIORecord()
    print(f"ConveyorPointIORecord retval is:{retval}")

    retval = robot.ConveyorPointARecord()
    print(f"ConveyorPointARecord retval is:{retval}")

    retval = robot.ConveyorRefPointRecord()
    print(f"ConveyorRefPointRecord retval is:{retval}")

    retval = robot.ConveyorPointBRecord()
    print(f"ConveyorPointBRecord retval is:{retval}")

    retval = robot.ConveyorStartEnd(0)
    print(f"ConveyorStartEnd retval is:{retval}")

    param = [1.0, 10000.0, 200.0, 0.0, 0.0, 20.0]
    retval = robot.ConveyorSetParam(param,0)
    print(f"ConveyorSetParam retval is:{retval}")

    cmp = [0.0, 0.0, 0.0]
    retval = robot.ConveyorCatchPointComp(cmp)
    print(f"ConveyorCatchPointComp retval is:{retval}")

    index = 1
    max_time = 30000
    block = 0
    retval = 0

    p1Desc = [-419.524, -13.000, 351.569, -178.118, 0.314, 3.833]
    p2Desc = [-321.222, 185.189, 335.520, -179.030, -1.284, -29.869]

    retval = robot.MoveCart(p1Desc, 1, 0, 100.0)
    print(f"MoveCart retval is:{retval}")

    retval = robot.WaitMs(1)
    print(f"WaitMs retval is:{retval}")

    retval = robot.ConveyorIODetect(10000)
    print(f"ConveyorIODetect retval is:{retval}")

    # retval = robot.ConveyorGetTrackData(1)
    # print(f"ConveyorGetTrackData retval is:{retval}")

    retval = robot.ConveyorTrackStart(1)
    print(f"ConveyorTrackStart retval is:{retval}")

    retval = robot.ConveyorTrackMoveL("cvrCatchPoint", 1, 0, 100)
    print(f"TrackMoveL retval is:{retval}")

    retval = robot.MoveGripper(index, 51, 40, 30, max_time, block, 0, 0, 0, 0)
    print(f"MoveGripper retval is:{retval}")

    retval = robot.ConveyorTrackMoveL("cvrRaisePoint", 1, 0, 100)
    print(f"TrackMoveL retval is:{retval}")

    retval = robot.ConveyorTrackEnd()
    print(f"ConveyorTrackEnd retval is:{retval}")

    robot.MoveCart(p2Desc, 1, 0, 100.0, 100.0)

    retval = robot.MoveGripper(index, 100, 40, 10, max_time, block, 0, 0, 0, 0)
    print(f"MoveGripper retval is:{retval}")

    robot.CloseRPC()


def TestAxleSensor(self):
    robot.AxleSensorConfig(18, 0, 0, 1)
    error, company, type = robot.AxleSensorConfigGet()
    print(f"company is:{company},type is:{type}")

    rtn = robot.AxleSensorActivate(1)
    print(f"AxleSensorActivate rtn is:{rtn}")

    time.sleep(1)

    rtn = robot.AxleSensorRegWrite(1, 4, 6, 1, 0, 0, 0)
    print(f"AxleSensorRegWrite rtn is:{rtn}")

    robot.CloseRPC()


def TestExDevProtocol(self):
    protocol = 4096
    rtn = robot.SetExDevProtocol(protocol)
    print(f"SetExDevProtocol rtn:{rtn}")

    rtn, protocol = robot.GetExDevProtocol()
    print(f"GetExDevProtocol rtn:{rtn},protocol is:{protocol}")

    robot.CloseRPC()




def TestAxleLua(self):
    robot.AxleLuaUpload("D://zUP/AXLE_LUA_End_DaHuan.lua")

    param = [7, 8, 1, 0, 5, 3, 1]  # 对应AxleComParam参数
    robot.SetAxleCommunicationParam(7, 8, 1, 0, 5, 3, 1)

    error,getParam0,getParam1,getParam2,getParam3,getParam4,getParam5,getParam6 = robot.GetAxleCommunicationParam()
    print(
        f"GetAxleCommunicationParam param is:{getParam0} {getParam1} {getParam2} {getParam3} {getParam4} {getParam5} {getParam6}")

    robot.SetAxleLuaEnable(1)
    error,luaEnableStatus = robot.GetAxleLuaEnableStatus()
    robot.SetAxleLuaEnableDeviceType(0, 1, 0)

    error,forceEnable, gripperEnable, ioEnable = robot.GetAxleLuaEnableDeviceType()
    print(f"GetAxleLuaEnableDeviceType param is:{forceEnable} {gripperEnable} {ioEnable}")

    func = [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1]
    robot.SetAxleLuaGripperFunc(1, func)
    error,getFunc = robot.GetAxleLuaGripperFunc(1)

    error,getforceEnable, getgripperEnable, getioEnable = robot.GetAxleLuaEnableDevice()

    print("\ngetforceEnable status:", end=" ")
    for i in range(8):
        print(f"{getforceEnable[i]},", end="")

    print("\ngetgripperEnable status:", end=" ")
    for i in range(8):
        print(f"{getgripperEnable[i]},", end="")

    print("\ngetioEnable status:", end=" ")
    for i in range(8):
        print(f"{getioEnable[i]},", end="")
    print()

    robot.ActGripper(1, 0)
    time.sleep(2)
    robot.ActGripper(1, 1)
    time.sleep(2)
    robot.MoveGripper(1, 90, 10, 100, 50000, 0, 0, 0, 0, 0)

    while True:
        error,pkg = robot.GetRobotRealTimeState()
        print(f"gripper pos is:{pkg.gripper_position}")
        time.sleep(0.1)

    robot.CloseRPC()

def testsmoot(self):
    while True:
        error, state = robot.GetSmarttoolBtnState()
        print(f"{state:016b}")
        time.sleep(0.1)


def TestSetWeldParam(self):
    robot.WeldingSetProcessParam(1, 177, 27, 1000, 178, 28, 176, 26, 1000)
    robot.WeldingSetProcessParam(2, 188, 28, 555, 199, 29, 133, 23, 333)

    start_current = 0
    start_voltage = 0
    start_time = 0
    weld_current = 0
    weld_voltage = 0
    end_current = 0
    end_voltage = 0
    end_time = 0

    error, start_current, start_voltage, start_time, weld_current, weld_voltage, end_current,end_voltage, end_time = robot.WeldingGetProcessParam(1)
    print(
        f"the Num 1 process param is {start_current} {start_voltage} {start_time} {weld_current} {weld_voltage} {end_current} {end_voltage} {end_time}")

    error, start_current, start_voltage, start_time, weld_current, weld_voltage, end_current,end_voltage, end_time = robot.WeldingGetProcessParam(2)
    print(
        f"the Num 2 process param is {start_current} {start_voltage} {start_time} {weld_current} {weld_voltage} {end_current} {end_voltage} {end_time}")

    rtn = robot.WeldingSetCurrentRelation(0, 400, 0, 10, 0)
    print(f"WeldingSetCurrentRelation rtn is: {rtn}")

    rtn = robot.WeldingSetVoltageRelation(0, 40, 0, 10, 1)
    print(f"WeldingSetVoltageRelation rtn is: {rtn}")

    current_min = 0
    current_max = 0
    vol_min = 0
    vol_max = 0
    output_vmin = 0
    output_vmax = 0
    cur_index = 0
    vol_index = 0

    rtn,current_min, current_max, output_vmin, output_vmax, cur_index = robot.WeldingGetCurrentRelation()
    print(f"WeldingGetCurrentRelation rtn is: {rtn}")
    print(
        f"current min {current_min} current max {current_max} output vol min {output_vmin} output vol max {output_vmax}")

    rtn,vol_min, vol_max, output_vmin, output_vmax, vol_index = robot.WeldingGetVoltageRelation()
    print(f"WeldingGetVoltageRelation rtn is: {rtn}")
    print(f"vol min {vol_min} vol max {vol_max} output vol min {output_vmin} output vol max {output_vmax}")

    rtn = robot.WeldingSetCurrent(1, 100, 0, 0)
    print(f"WeldingSetCurrent rtn is: {rtn}")

    time.sleep(3)

    rtn = robot.WeldingSetVoltage(1, 10, 0, 0)
    print(f"WeldingSetVoltage rtn is: {rtn}")

    rtn = robot.WeaveSetPara(0, 0, 2.000000, 0, 10.000000, 0.000000, 0.000000, 0, 0, 0, 0, 0,0.0, 60.000000)
    print(f"rtn is: {rtn}")

    robot.WeaveOnlineSetPara(0, 0, 1, 0, 20, 0, 0, 0, 0)

    rtn = robot.WeldingSetCheckArcInterruptionParam(1, 200)
    print(f"WeldingSetCheckArcInterruptionParam {rtn}")

    rtn = robot.WeldingSetReWeldAfterBreakOffParam(1, 5.7, 98.2, 0)
    print(f"WeldingSetReWeldAfterBreakOffParam {rtn}")

    enable = 0
    length = 0
    velocity = 0
    move_type = 0
    check_enable = 0
    arc_interrupt_time_length = 0

    rtn,check_enable, arc_interrupt_time_length = robot.WeldingGetCheckArcInterruptionParam()
    print(
        f"WeldingGetCheckArcInterruptionParam checkEnable {check_enable} arcInterruptTimeLength {arc_interrupt_time_length}")

    rtn,enable, length, velocity, move_type = robot.WeldingGetReWeldAfterBreakOffParam()
    print(
        f"WeldingGetReWeldAfterBreakOffParam enable = {enable}, length = {length}, velocity = {velocity}, moveType = {move_type}")

    robot.SetWeldMachineCtrlModeExtDoNum(17)
    for i in range(5):
        robot.SetWeldMachineCtrlMode(0)
        time.sleep(1)
        robot.SetWeldMachineCtrlMode(1)
        time.sleep(1)

    robot.CloseRPC()

def TestWelding(self):
    robot.WeldingSetCurrent(1, 230, 0, 0)
    robot.WeldingSetVoltage(1, 24, 0, 1)

    p1Desc = [228.879, -503.594, 453.984, -175.580, 8.293, 171.267]
    p1Joint = [102.700, -85.333, 90.518, -102.365, -83.932, 22.134]

    p2Desc = [-333.302, -435.580, 449.866, -174.997, 2.017, 109.815]
    p2Joint = [41.862, -85.333, 90.526, -100.587, -90.014, 22.135]

    exaxisPos = [0, 0, 0, 0]
    offdese = [0, 0, 0, 0, 0, 0]

    robot.MoveJ(joint_pos=p1Joint, tool=13, user=0, vel=20)
    robot.ARCStart(1, 0, 10000)
    robot.WeaveStart(0)
    robot.MoveL(desc_pos=p2Desc, tool=13, user=0, vel=20)
    robot.ARCEnd(1, 0, 10000)
    robot.WeaveEnd(0)

    robot.CloseRPC()


def TestSegWeld(self):
    robot.WeldingSetCurrent(1, 230, 0, 0)
    robot.WeldingSetVoltage(1, 24, 0, 1)

    p1Desc = [228.879, -503.594, 453.984, -175.580, 8.293, 171.267]
    p1Joint = [102.700, -85.333, 90.518, -102.365, -83.932, 22.134]

    p2Desc = [-333.302, -435.580, 449.866, -174.997, 2.017, 109.815]
    p2Joint = [41.862, -85.333, 90.526, -100.587, -90.014, 22.135]

    exaxisPos = [0, 0, 0, 0]
    offdese = [0, 0, 0, 0, 0, 0]

    rtn = robot.SegmentWeldStart(p1Desc, p2Desc, p1Joint, p2Joint, 20, 20, 0, 0, 5000, 0, 0, 0, 0)
    print(f"SegmentWeldStart rtn is {rtn}")

    robot.CloseRPC()


def TestWeave(self):
    p1Desc = [228.879, -503.594, 453.984, -175.580, 8.293, 171.267]
    p1Joint = [102.700, -85.333, 90.518, -102.365, -83.932, 22.134]

    p2Desc = [-333.302, -435.580, 449.866, -174.997, 2.017, 109.815]
    p2Joint = [41.862, -85.333, 90.526, -100.587, -90.014, 22.135]

    exaxisPos = [0, 0, 0, 0]
    offdese = [0, 0, 0, 0, 0, 0]

    robot.MoveJ(joint_pos= p1Joint,tool= 13,user= 0)
    robot.WeaveStartSim(0)
    robot.MoveL(desc_pos= p2Desc,tool= 13,user= 0)
    robot.WeaveEndSim(0)
    robot.MoveJ(joint_pos= p1Joint,tool= 13,user= 0)
    robot.WeaveInspectStart(0)
    robot.MoveL(desc_pos= p2Desc,tool= 13,user= 0,)
    robot.WeaveInspectEnd(0)

    robot.WeldingSetVoltage(1, 19, 0, 0)
    robot.WeldingSetCurrent(1, 190, 0, 0)
    robot.MoveL(desc_pos= p1Desc,tool= 1,user= 1,vel= 100,acc= 100,ovl= 50)
    robot.ARCStart(1, 0, 10000)
    robot.ArcWeldTraceControl(1, 0, 1, 0.06, 5, 5, 60, 1, 0.06, 5, 5, 80, 0, 0, 4, 1, 10, 0, 0)
    robot.WeaveStart(0)
    robot.WeaveChangeStart(1, 0, 50, 30)
    robot.MoveL(desc_pos= p2Desc,tool= 1,user= 1,vel= 100)
    robot.WeaveChangeEnd()
    robot.WeaveEnd(0)
    robot.ArcWeldTraceControl(0, 0, 1, 0.06, 5, 5, 60, 1, 0.06, 5, 5, 80, 0, 0, 4, 1, 10, 0, 0)
    robot.ARCEnd(1, 0, 10000)

    robot.CloseRPC()


import time


def TestWelding(self):
    # Wire feed tests
    robot.SetForwardWireFeed(0, 1)
    time.sleep(1)
    robot.SetForwardWireFeed(0, 0)
    robot.SetReverseWireFeed(0, 1)
    time.sleep(1)
    robot.SetReverseWireFeed(0, 0)
    robot.SetAspirated(0, 1)
    time.sleep(1)
    robot.SetAspirated(0, 0)
    # Welding parameters setup
    robot.WeldingSetCurrent(1, 230, 0, 0)
    robot.WeldingSetVoltage(1, 24, 0, 1)
    # Position definitions
    p1Desc = [228.879, -503.594, 453.984, -175.580, 8.293, 171.267]
    p1Joint = [102.700, -85.333, 90.518, -102.365, -83.932, 22.134]
    p2Desc = [-333.302, -435.580, 449.866, -174.997, 2.017, 109.815]
    p2Joint = [41.862, -85.333, 90.526, -100.587, -90.014, 22.135]
    exaxisPos = [0, 0, 0, 0]
    offdese = [0, 0, 0, 0, 0, 0]
    # Movement and welding sequence
    robot.MoveJ(joint_pos=p1Joint, tool=13, user=0)
    robot.ARCStart(1, 0, 10000)
    robot.WeaveStart(0)
    robot.MoveL(desc_pos=p2Desc, tool=13, user=0)
    robot.ARCEnd(1, 0, 10000)
    robot.WeaveEnd(0)
    # Welding control functions
    robot.WeldingStartReWeldAfterBreakOff()
    robot.WeldingAbortWeldAfterBreakOff()
    robot.CloseRPC()

def TestSSHMd5(self):
    file_path = "/fruser/airlab.lua"
    md5 = ""
    emerg_state = 0
    si0_state = 0
    si1_state = 0
    sdk_com_state = 0

    ssh_keygen = ""
    retval,ssh_keygen = robot.GetSSHKeygen()
    print(f"GetSSHKeygen retval is: {retval}")
    print(f"ssh key is: {ssh_keygen}")

    ssh_name = "fr"
    ssh_ip = "192.168.58.45"
    ssh_route = "/home/fr"
    ssh_robot_url = "/root/robot/dhpara.config"
    retval = robot.SetSSHScpCmd(1, ssh_name, ssh_ip, ssh_route, ssh_robot_url)
    print(f"SetSSHScpCmd retval is: {retval}")
    print(f"robot url is: {ssh_robot_url}")

    error, md5 = robot.ComputeFileMD5(file_path)
    print(f"md5 is: {md5}")

    robot.CloseRPC()


def TestRealtimePeriod(self):
    robot.SetRobotRealtimeStateSamplePeriod(10)
    error,getPeriod = robot.GetRobotRealtimeStateSamplePeriod()
    print(f"period is {getPeriod}")
    time.sleep(1)

    robot.CloseRPC()


def TestUpgrade(self):
    error = robot.SoftwareUpgrade("D://zUP/QNX382/software.tar.gz", False)
    print(f"SoftwareUpgrade error is {error}")
    while True:
        curState = robot.GetSoftwareUpgradeState()
        print(f"upgrade state is {curState}")
        time.sleep(3)

    robot.CloseRPC()

def TestPointTable(self):
    save_path = "D://zDOWN/"
    point_table_name = "point_table_FR5.db"
    rtn = robot.PointTableDownLoad(point_table_name, save_path)
    print(f"download : {point_table_name} fail: {rtn}")

    upload_path = "D://zDOWN/point_table_FR5.db"
    rtn = robot.PointTableUpLoad(upload_path)
    print(f"retval is: {rtn}")

    point_tablename = "point_table_FR5.db"
    lua_name = "test0610.lua"
    rtn,error = robot.PointTableUpdateLua(point_tablename, lua_name)
    print(f"retval is: {rtn}")

    robot.CloseRPC()

def TestDownLoadRobotData(self):
    rtn = robot.RbLogDownload("D://zDOWN/")
    print(f"RbLogDownload rtn is {rtn}")

    rtn = robot.AllDataSourceDownload("D://zDOWN/")
    print(f"AllDataSourceDownload rtn is {rtn}")

    rtn = robot.DataPackageDownload("D://zDOWN/")
    print(f"DataPackageDownload rtn is {rtn}")

    robot.CloseRPC()

def TestExtDIConfig(self):
    rtn = robot.SetArcStartExtDoNum(10)
    print(f"SetArcStartExtDoNum rtn is {rtn}")
    rtn = robot.SetAirControlExtDoNum(20)
    print(f"SetAirControlExtDoNum rtn is {rtn}")
    rtn = robot.SetWireForwardFeedExtDoNum(30)
    print(f"SetWireForwardFeedExtDoNum rtn is {rtn}")
    rtn = robot.SetWireReverseFeedExtDoNum(40)

    rtn = robot.SetWeldReadyExtDiNum(50)
    print(f"SetWeldReadyExtDiNum rtn is {rtn}")
    rtn = robot.SetArcDoneExtDiNum(60)
    print(f"SetArcDoneExtDiNum rtn is {rtn}")
    rtn = robot.SetExtDIWeldBreakOffRecover(70, 80)
    print(f"SetExtDIWeldBreakOffRecover rtn is {rtn}")
    rtn = robot.SetWireSearchExtDIONum(0, 1)
    print(f"SetWireSearchExtDIONum rtn is {rtn}")

    robot.CloseRPC()



def Test485Auxservo(self):
    retval = robot.AuxServoSetParam(1, 1, 1, 1, 131072, 15.45)
    print(f"AuxServoSetParam is: {retval}")

    servoCompany = 0
    servoModel = 0
    servoSoftVersion = 0
    servoResolution = 0
    axisMechTransRatio = 0.0

    retval, servoCompany, servoModel, servoSoftVersion, servoResolution, axisMechTransRatio = robot.AuxServoGetParam(1)
    print(f"servoCompany {servoCompany}\n"
          f"servoModel {servoModel}\n"
          f"servoSoftVersion {servoSoftVersion}\n"
          f"servoResolution {servoResolution}\n"
          f"axisMechTransRatio {axisMechTransRatio}\n")

    retval = robot.AuxServoSetParam(1, 10, 11, 12, 13, 14)
    print(f"AuxServoSetParam is: {retval}")

    retval, servoCompany, servoModel, servoSoftVersion, servoResolution, axisMechTransRatio = robot.AuxServoGetParam(1)
    print(f"servoCompany {servoCompany}\n"
          f"servoModel {servoModel}\n"
          f"servoSoftVersion {servoSoftVersion}\n"
          f"servoResolution {servoResolution}\n"
          f"axisMechTransRatio {axisMechTransRatio}\n")

    retval = robot.AuxServoSetParam(1, 1, 1, 1, 131072, 36)
    print(f"AuxServoSetParam is: {retval}")
    time.sleep(3)

    robot.AuxServoSetAcc(3000, 3000)
    robot.AuxServoSetEmergencyStopAcc(5000, 5000)
    time.sleep(1)

    emagacc = 0.0
    emagdec = 0.0
    acc = 0.0
    dec = 0.0
    error,emagacc, emagdec = robot.AuxServoGetEmergencyStopAcc()
    print(f"emergency acc is {emagacc}  dec is {emagdec}")
    error,acc, dec = robot.AuxServoGetAcc()
    print(f"acc is {acc}  dec is {dec}")

    robot.AuxServoSetControlMode(1, 0)
    time.sleep(2)

    retval = robot.AuxServoEnable(1, 0)
    print(f"AuxServoEnable disenable {retval}")
    time.sleep(1)

    servoErrCode = 0
    servoState = 0
    servoPos = 0.0
    servoSpeed = 0.0
    servoTorque = 0.0
    retval, servoErrCode, servoState, servoPos, servoSpeed, servoTorque = robot.AuxServoGetStatus(1)
    print(f"AuxServoGetStatus servoState {servoState}")
    time.sleep(1)

    retval = robot.AuxServoEnable(1, 1)
    print(f"AuxServoEnable enable {retval}")
    time.sleep(1)
    retval, servoErrCode, servoState, servoPos, servoSpeed, servoTorque = robot.AuxServoGetStatus(1)
    print(f"AuxServoGetStatus servoState {servoState}")
    time.sleep(1)

    retval = robot.AuxServoHoming(1, 1, 5, 1,100)
    print(f"AuxServoHoming {retval}")
    time.sleep(3)

    retval = robot.AuxServoSetTargetPos(1, 200, 30,100)
    print(f"AuxServoSetTargetPos {retval}")
    time.sleep(1)
    retval, servoErrCode, servoState, servoPos, servoSpeed, servoTorque = robot.AuxServoGetStatus(1)
    print(f"AuxServoGetStatus servoSpeed {servoSpeed}")
    time.sleep(8)

    robot.AuxServoSetControlMode(1, 1)
    time.sleep(2)

    robot.AuxServoEnable(1, 0)
    time.sleep(1)
    robot.AuxServoEnable(1, 1)
    time.sleep(1)
    robot.AuxServoSetTargetSpeed(1, 100, 80)

    time.sleep(5)
    robot.AuxServoSetTargetSpeed(1, 0, 80)

    robot.CloseRPC()


def TestArcWeldTrace(self):
    mulitilineorigin1_joint = [-24.090, -63.501, 84.288, -111.940, -93.426, 57.669]
    mulitilineorigin1_desc = [-677.559, 190.951, -1.205, 1.144, -41.482, -82.577]

    mulitilineX1_desc = [-677.556, 211.949, -1.206]
    mulitilineZ1_desc = [-677.564, 190.956, 19.817]

    mulitilinesafe_joint = [-25.734, -63.778, 81.502, -108.975, -93.392, 56.021]
    mulitilinesafe_desc = [-677.561, 211.950, 19.812, 1.144, -41.482, -82.577]
    mulitilineorigin2_joint = [-29.743, -75.623, 101.241, -116.354, -94.928, 55.735]
    mulitilineorigin2_desc = [-563.961, 215.359, -0.681, 2.845, -40.476, -87.443]

    mulitilineX2_desc = [-563.965, 220.355, -0.680]
    mulitilineZ2_desc = [-563.968, 215.362, 4.331]

    epos = [0, 0, 0, 0]
    offset = [0, 0, 0, 0, 0, 0]

    time.sleep(0.01)

    error = robot.MoveJ(joint_pos= mulitilinesafe_joint,tool= 13,user= 0,vel= 10)
    print(f"MoveJ return: {error}")
    error = robot.MoveL(desc_pos= mulitilineorigin1_desc,tool= 13,user= 0,vel= 10,speedPercent=100)
    print(f"MoveL return: {error}")
    error = robot.MoveJ(joint_pos= mulitilinesafe_joint,tool= 13,user= 0,vel= 10)
    print(f"MoveJ return: {error}")
    error = robot.MoveL(desc_pos= mulitilineorigin2_desc,tool= 13,user= 0,vel= 10,speedPercent=100)
    print(f"MoveL return: {error}")
    error = robot.MoveJ(joint_pos= mulitilinesafe_joint,tool= 13,user= 0,vel= 10)
    print(f"MoveJ return: {error}")
    error = robot.MoveL(desc_pos= mulitilineorigin1_desc,tool= 13,user= 0,vel= 10,speedPercent=100)
    print(f"MoveL return: {error}")

    error = robot.ARCStart(1, 0, 3000)
    print(f"ARCStart return: {error}")
    error = robot.WeaveStart(0)
    print(f"WeaveStart return: {error}")
    error = robot.ArcWeldTraceControl(1, 0, 1, 0.06, 5, 5, 50, 1, 0.06, 5, 5, 55, 0, 0, 4, 1, 10,0,0)
    print(f"ArcWeldTraceControl return: {error}")
    error = robot.MoveL(desc_pos= mulitilineorigin2_desc,tool= 13,user= 0,vel= 1,speedPercent=100)
    print(f"MoveL return: {error}")
    error = robot.ArcWeldTraceControl(0, 0, 1, 0.06, 5, 5, 50, 1, 0.06, 5, 5, 55, 0, 0, 4, 1, 10,0,0)
    print(f"ArcWeldTraceControl return: {error}")
    error = robot.WeaveEnd(0)
    print(f"WeaveEnd return: {error}")
    error = robot.ARCEnd(1, 0, 10000)
    print(f"ARCEnd return: {error}")
    error = robot.MoveJ(joint_pos= mulitilinesafe_joint,tool= 13,user= 0,vel= 10)
    print(f"MoveJ return: {error}")

    error,offset = robot.MultilayerOffsetTrsfToBase(mulitilineorigin1_desc[:3], mulitilineX1_desc, mulitilineZ1_desc, 10.0, 0.0, 0.0)
    print(f"MultilayerOffsetTrsfToBase return: {error}")
    error = robot.MoveL(desc_pos= mulitilineorigin1_desc,tool= 13,user= 0,vel= 10,speedPercent=100)
    print(f"MoveL return: {error}")
    error = robot.ARCStart(1, 0, 3000)
    print(f"ARCStart return: {error}")

    error, offset = robot.MultilayerOffsetTrsfToBase(mulitilineorigin2_desc[:3], mulitilineX2_desc, mulitilineZ2_desc, 10, 0, 0)
    print(f"MultilayerOffsetTrsfToBase return: {error}")
    error = robot.ArcWeldTraceReplayStart()
    print(f"ArcWeldTraceReplayStart return: {error}")
    error = robot.MoveL(desc_pos= mulitilineorigin2_desc,tool= 13,user= 0,vel= 2,speedPercent=100)
    print(f"MoveL return: {error}")
    error = robot.ArcWeldTraceReplayEnd()
    print(f"ArcWeldTraceReplayEnd return: {error}")
    error = robot.ARCEnd(1, 0, 10000)
    print(f"ARCEnd return: {error}")

    error = robot.MoveJ(joint_pos= mulitilinesafe_joint,tool= 13,user= 0,vel= 10)
    print(f"MoveJ return: {error}")

    error, offset = robot.MultilayerOffsetTrsfToBase(mulitilineorigin1_desc[:3], mulitilineX1_desc, mulitilineZ1_desc, 0, 10, 0)
    print(f"MultilayerOffsetTrsfToBase return: {error}")
    error = robot.MoveL(desc_pos= mulitilineorigin1_desc,tool= 13,user= 0,vel= 10,speedPercent=100)
    print(f"MoveL return: {error}")
    error = robot.ARCStart(1, 0, 3000)
    print(f"ARCStart return: {error}")

    error, offset = robot.MultilayerOffsetTrsfToBase(mulitilineorigin2_desc[:3], mulitilineX2_desc, mulitilineZ2_desc, 0, 10, 0)
    error = robot.ArcWeldTraceReplayStart()
    print(f"ArcWeldTraceReplayStart return: {error}")
    error = robot.MoveL(desc_pos= mulitilineorigin2_desc,tool= 13,user= 0,vel= 2,speedPercent=100)
    print(f"MoveL return: {error}")
    error = robot.ArcWeldTraceReplayEnd()
    print(f"ArcWeldTraceReplayEnd return: {error}")
    error = robot.ARCEnd(1, 0, 3000)
    print(f"ARCEnd return: {error}")

    error = robot.MoveJ(joint_pos= mulitilinesafe_joint,tool= 13,user= 0,vel= 10)
    print(f"MoveJ return: {error}")

    robot.CloseRPC()

def TestWireSearch(self):
    toolCoord = [0, 0, 200, 0, 0, 0]
    robot.SetToolCoord(1, toolCoord, 0, 0, 1, 0)

    wobjCoord = [0, 0, 0, 0, 0, 0]
    robot.SetWObjCoord(1, wobjCoord, 0)

    # robot.ExtDevSetUDPComParam("192.168.58.88", 2021, 2, 50, 5, 50, 1, 50, 10)
    # robot.ExtDevLoadUDPDriver()
    #
    # robot.SetWireSearchExtDIONum(0, 0)

    exaxisPos = [0, 0, 0, 0]
    offdese = [0, 0, 0, 0, 0, 0]

    descStart = [216.543, 445.175, 93.465, 179.683, 1.757, -112.641]
    jointStart = [-128.345, -86.660, 114.679, -119.625, -89.219, 74.303]

    descEnd = [111.143, 523.384, 87.659, 179.703, 1.835, -97.750]
    jointEnd = [-113.454, -81.060, 109.328, -119.954, -89.218, 74.302]

    error = robot.MoveL(desc_pos=descStart,tool= 1,user= 1,vel= 100)
    print(f"MoveL return: {error}")
    error = robot.MoveL(desc_pos=descEnd,tool= 1,user= 1,vel= 100)
    print(f"MoveL return: {error}")

    descREF0A = [142.135, 367.604, 86.523, 179.728, 1.922, -111.089]
    jointREF0A = [-126.794, -100.834, 128.922, -119.864, -89.218, 74.302]

    descREF0B = [254.633, 463.125, 72.604, 179.845, 2.341, -114.704]
    jointREF0B = [-130.413, -81.093, 112.044, -123.163, -89.217, 74.303]

    descREF1A = [92.556, 485.259, 47.476, -179.932, 3.130, -97.512]
    jointREF1A = [-113.231, -83.815, 119.877, -129.092, -89.217, 74.303]

    descREF1B = [203.103, 583.836, 63.909, 179.991, 2.854, -103.372]
    jointREF1B = [-119.088, -69.676, 98.692, -121.761, -89.219, 74.303]

    error = robot.WireSearchStart(0, 10, 100, 0, 10, 100, 0)
    print(f"WireSearchStart return: {error}")
    error = robot.MoveL(desc_pos=descREF0A,tool= 1,user= 1,vel= 100)
    print(f"MoveL return: {error}")
    error = robot.MoveL(desc_pos=descREF0B,tool= 1,user= 1,vel= 100,search=1)
    print(f"MoveL return: {error}")
    error = robot.WireSearchWait("REF0")
    print(f"WireSearchWait return: {error}")
    error = robot.WireSearchEnd(0, 10, 100, 0, 10, 100, 0)
    print(f"WireSearchEnd return: {error}")

    error = robot.WireSearchStart(0, 10, 100, 0, 10, 100, 0)
    print(f"WireSearchStart return: {error}")
    error = robot.MoveL(desc_pos= descREF1A,tool= 1,user= 1,vel= 100)
    print(f"MoveL return: {error}")
    error = robot.MoveL(desc_pos= descREF1B,tool= 1,user= 1,vel= 100,search=1)
    print(f"MoveL return: {error}")
    error = robot.WireSearchWait("REF1")
    print(f"WireSearchWait return: {error}")
    error = robot.WireSearchEnd(0, 10, 100, 0, 10, 100, 0)

    error = robot.WireSearchStart(0, 10, 100, 0, 10, 100, 0)
    print(f"WireSearchStart return: {error}")
    error = robot.MoveL(desc_pos= descREF0A,tool= 1,user= 1,vel= 100)
    print(f"MoveL return: {error}")
    error = robot.MoveL(desc_pos= descREF0B,tool= 1,user= 1,vel= 100,search=1)
    print(f"MoveL return: {error}")
    error = robot.WireSearchWait("RES0")
    print(f"WireSearchWait return: {error}")
    error = robot.WireSearchEnd(0, 10, 100, 0, 10, 100, 0)
    print(f"WireSearchEnd return: {error}")
    error = robot.WireSearchStart(0, 10, 100, 0, 10, 100, 0)
    print(f"WireSearchStart return: {error}")
    error = robot.MoveL(desc_pos= descREF1A,tool= 1,user= 1,vel= 100)
    print(f"MoveL return: {error}")
    error = robot.MoveL(desc_pos= descREF1B,tool= 1,user= 1,vel= 100,search=1)
    print(f"MoveL return: {error}")
    error = robot.WireSearchWait("RES1")
    print(f"WireSearchWait return: {error}")
    error = robot.WireSearchEnd(0, 10, 100, 0, 10, 100, 0)
    print(f"WireSearchEnd return: {error}")
    varNameRef = ["REF0", "REF1", "#", "#", "#", "#"]
    varNameRes = ["RES0", "RES1", "#", "#", "#", "#"]
    offectFlag = 0
    offectPos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    error, offectFlag, offectPos = robot.GetWireSearchOffset(0, 0, varNameRef, varNameRes)
    print(f"GetWireSearchOffset return: {error}")
    error = robot.PointsOffsetEnable(0, offectPos)
    print(f"PointsOffsetEnable return: {error}")
    error = robot.MoveL(desc_pos= descStart,tool= 1,user= 1,vel= 100)
    print(f"MoveL return: {error}")
    error = robot.MoveL(desc_pos= descEnd,tool= 1,user= 1,vel= 100,search=1)
    print(f"MoveL return: {error}")
    error = robot.PointsOffsetDisable()

    robot.CloseRPC()

def TestFTInit(self):
    company = 24
    device = 0
    softversion = 0
    bus = 1
    index = 1

    robot.FT_SetConfig(company, device, softversion, bus)
    time.sleep(1)

    error,[company, device, softversion, bus] = robot.FT_GetConfig()
    print(f"FT config:{company},{device},{softversion},{bus}")
    time.sleep(1)

    robot.FT_Activate(0)
    time.sleep(1)
    robot.FT_Activate(1)
    time.sleep(1)

    time.sleep(1)
    robot.FT_SetZero(0)
    time.sleep(1)

    error,ft = robot.FT_GetForceTorqueOrigin()
    print(f"ft origin:{ft[0]},{ft[1]},{ft[2]},{ft[3]},{ft[4]},{ft[5]}")

    robot.FT_SetZero(1)
    time.sleep(1)

    ftCoord = [0, 0, 0, 0, 0, 0]
    robot.FT_SetRCS(0, ftCoord)

    robot.SetForceSensorPayload(0.824)
    robot.SetForceSensorPayloadCog(0.778, 2.554, 48.765)

    error,weight = robot.GetForceSensorPayload()
    error,x, y, z = robot.GetForceSensorPayloadCog()
    print(f"the FT load is  {weight}, {x} {y} {z}")

    robot.SetForceSensorPayload(0)
    robot.SetForceSensorPayloadCog(0, 0, 0)

    error,computeWeight, tran = robot.ForceSensorAutoComputeLoad()
    # print(f"ForceSensorAutoComputeLoad return {error}")
    print(f"the result is weight {computeWeight} pos is {tran[0]} {tran[1]} {tran[2]}")

    robot.CloseRPC()


import time


import time


def TestUDPAxis(self):
    # 设置UDP通信参数
    rtn = robot.ExtDevSetUDPComParam("192.168.58.88", 2021, 2, 100, 3, 200, 1, 100, 5, 1)
    print(f"ExtDevSetUDPComParam rtn is {rtn}")
    # 获取UDP通信参数
    ip = ""
    port = 0
    period = 0
    lossPkgTime = 0
    lossPkgNum = 0
    disconnectTime = 0
    reconnectEnable = 0
    reconnectPeriod = 0
    reconnectNum = 0
    rtn,[ip, port, period, lossPkgTime, lossPkgNum,disconnectTime, reconnectEnable, reconnectPeriod, reconnectNum] = robot.ExtDevGetUDPComParam()
    param_str = (f"\nip {ip}\nport {port}\nperiod {period}\nlossPkgTime {lossPkgTime}"
                 f"\nlossPkgNum {lossPkgNum}\ndisConntime {disconnectTime}"
                 f"\nreconnecable {reconnectEnable}\nreconnperiod {reconnectPeriod}"
                 f"\nreconnnun {reconnectNum}")
    print(f"ExtDevGetUDPComParam rtn is {rtn}{param_str}")
    # 加载UDP驱动
    robot.ExtDevLoadUDPDriver()
    # 伺服使能
    rtn = robot.ExtAxisServoOn(1, 1)
    print(f"ExtAxisServoOn axis id 1 rtn is {rtn}")
    rtn = robot.ExtAxisServoOn(2, 1)
    print(f"ExtAxisServoOn axis id 2 rtn is {rtn}")
    time.sleep(2)
    # 设置回零参数
    robot.ExtAxisSetHoming(1, 0, 10, 2)
    time.sleep(2)
    rtn = robot.ExtAxisSetHoming(2, 0, 10, 2)
    print(f"ExtAxisSetHoming rtn is {rtn}")
    time.sleep(4)
    # 设置轴参数
    rtn = robot.SetRobotPosToAxis(1)
    print(f"SetRobotPosToAxis rtn is {rtn}")
    rtn = robot.SetAxisDHParaConfig(10, 20, 0, 0, 0, 0, 0, 0, 0)
    print(f"SetAxisDHParaConfig rtn is {rtn}")
    rtn = robot.ExtAxisParamConfig(1, 1, 1, 1000, -1000, 1000, 1000, 1.905, 262144, 200, 1, 0, 0)
    print(f"ExtAxisParamConfig axis 1 rtn is {rtn}")
    rtn = robot.ExtAxisParamConfig(2, 1, 1, 1000, -1000, 1000, 1000, 4.444, 262144, 200, 1, 0, 0)
    print(f"ExtAxisParamConfig axis 2 rtn is {rtn}")
    time.sleep(3)
    # 轴1点动测试
    robot.ExtAxisStartJog(1, 0, 10, 10, 30)
    time.sleep(1)
    robot.ExtAxisStopJog(1)
    time.sleep(3)
    robot.ExtAxisServoOn(1, 0)
    # 轴2点动测试
    time.sleep(3)
    robot.ExtAxisStartJog(2, 0, 10, 10, 30)
    time.sleep(1)
    robot.ExtAxisStopJog(2)
    time.sleep(3)
    robot.ExtAxisServoOn(2, 0)
    # 卸载驱动并关闭连接
    robot.ExtDevUnloadUDPDriver()
    robot.CloseRPC()

def TestFTLoadCompute(self):
    company = 24
    device = 0
    softversion = 0
    bus = 1
    index = 1

    robot.FT_SetConfig(company, device, softversion, bus)
    time.sleep(1)

    error,[company, device, softversion, bus] = robot.FT_GetConfig()
    print(f"FT config:{company},{device},{softversion},{bus}")
    time.sleep(1)

    robot.FT_Activate(0)
    time.sleep(1)
    robot.FT_Activate(1)
    time.sleep(1)

    robot.FT_SetZero(0)
    time.sleep(1)

    error,ft = robot.FT_GetForceTorqueOrigin()
    print(f"ft origin:{ft[0]},{ft[1]},{ft[2]},{ft[3]},{ft[4]},{ft[5]}")

    robot.FT_SetZero(1)
    time.sleep(1)

    tcoord = [0, 0, 35.0, 0, 0, 0]
    robot.SetToolCoord(10, tcoord, 1, 0, 0, 0)

    robot.FT_PdIdenRecord(10)
    time.sleep(1)

    error,weight = robot.FT_PdIdenCompute()
    print(f"payload weight:{weight}")

    desc_p1 = [-419.524, -13.000, 351.569, -178.118, 0.314, 3.833]
    desc_p2 = [-321.222, 185.189, 335.520, -179.030, -1.284, -29.869]
    desc_p3 = [-327.622, 402.230, 320.402, -178.067, 2.127, -46.207]

    robot.MoveCart(desc_p1, 0, 0, 100.0)
    time.sleep(1)
    robot.FT_PdCogIdenRecord(10, 1)

    robot.MoveCart(desc_p2, 0, 0, 100.0)
    time.sleep(1)
    robot.FT_PdCogIdenRecord(10, 2)

    robot.MoveCart(desc_p3, 0, 0, 100.0)
    time.sleep(1)
    robot.FT_PdCogIdenRecord(10, 3)
    time.sleep(1)

    error,cog = robot.FT_PdCogIdenCompute()
    print(f"FT_PdCogIdenCompute return {error}")
    print(f"cog:{cog[0]},{cog[1]},{cog[2]}")

    robot.CloseRPC()


def TestFTGuard(self):
    company = 24
    device = 0
    softversion = 0
    bus = 1
    index = 1

    robot.FT_SetConfig(company, device, softversion, bus)
    time.sleep(1)

    error,[company, device, softversion, bus] = robot.FT_GetConfig()
    print(f"FT config:{company},{device},{softversion},{bus}")
    time.sleep(1)

    robot.FT_Activate(0)
    time.sleep(1)
    robot.FT_Activate(1)
    time.sleep(1)

    robot.FT_SetZero(0)
    time.sleep(1)

    sensor_id = 1
    select = [1, 1, 1, 1, 1, 1]
    max_threshold = [10.0, 10.0, 10.0, 10.0, 10.0, 10.0]
    min_threshold = [5.0, 5.0, 5.0, 5.0, 5.0, 5.0]

    ft = None  # Will be filled by FT_Guard call if needed

    desc_p1 = [-419.524, -13.000, 351.569, -178.118, 0.314, 3.833]
    desc_p2 = [-321.222, 185.189, 335.520, -179.030, -1.284, -29.869]
    desc_p3 = [-327.622, 402.230, 320.402, -178.067, 2.127, -46.207]

    error = robot.FT_Guard(1, sensor_id, select,[0.0,0.0,0.0,0.0,0.0,0.0], max_threshold, min_threshold)

    robot.MoveCart(desc_p1, 0, 0, 100.0)
    robot.MoveCart(desc_p2, 0, 0, 100.0)
    robot.MoveCart(desc_p3, 0, 0, 100.0)

    robot.FT_Guard(0, sensor_id, select,[0.0,0.0,0.0,0.0,0.0,0.0], max_threshold, min_threshold)

    robot.CloseRPC()

def TestFTControl(self):
    company = 24
    device = 0
    softversion = 0
    bus = 1
    index = 1

    robot.FT_SetConfig(company, device, softversion, bus)
    time.sleep(1)

    error,[company, device, softversion, bus] = robot.FT_GetConfig()
    print(f"FT config:{company},{device},{softversion},{bus}")
    time.sleep(1)

    robot.FT_Activate(0)
    time.sleep(1)
    robot.FT_Activate(1)
    time.sleep(1)

    robot.FT_SetZero(0)
    time.sleep(1)

    sensor_id = 1
    select = [0, 0, 1, 0, 0, 0]
    ft_pid = [0.0005, 0.0, 0.0, 0.0, 0.0, 0.0]
    adj_sign = 0
    ILC_sign = 0
    max_dis = 100.0
    max_ang = 0.0

    # ForceTorque input: Only Fz active
    ft = [0.0,0.0,-10.0,0.0,0.0,0.0]

    # Define positions
    j1 = [-11.904, -99.669, 117.473, -108.616, -91.726, 74.256]
    j2 = [-45.615, -106.172, 124.296, -107.151, -91.282, 74.255]
    desc_p1 = [-419.524, -13.000, 351.569, -178.118, 0.314, 3.833]
    desc_p2 = [-321.222, 185.189, 335.520, -179.030, -1.284, -29.869]
    epos = [0, 0, 0, 0]
    offset_pos = [0, 0, 0, 0, 0, 0]

    # Move to initial joint position
    rtn = robot.MoveJ(joint_pos=j1,tool= 0,user= 0,vel= 100.0)

    # Enable FT Control (force control on Fz)
    robot.FT_Control(1, sensor_id, select, ft, ft_pid, adj_sign, ILC_sign, max_dis, max_ang)

    # Move linearly while under force control
    rtn = robot.MoveJ(joint_pos=j2,tool= 0,user= 0,vel= 100.0)

    # Disable FT Control
    robot.FT_Control(0, sensor_id, select, ft, ft_pid, adj_sign, ILC_sign, max_dis, max_ang)

    robot.CloseRPC()

def TestFTSearch(self):
    company = 24
    device = 0
    softversion = 0
    bus = 1
    index = 1

    robot.FT_SetConfig(company, device, softversion, bus)
    time.sleep(1)

    error,[company, device, softversion, bus] = robot.FT_GetConfig()
    print(f"FT config:{company},{device},{softversion},{bus}")
    time.sleep(1)

    robot.FT_Activate(0)
    time.sleep(1)
    robot.FT_Activate(1)
    time.sleep(1)

    robot.FT_SetZero(0)
    time.sleep(1)

    # Constant force control parameters
    status = 1
    sensor_num = 1
    gain = [0.0001, 0.0, 0.0, 0.0, 0.0, 0.0]
    adj_sign = 0
    ILC_sign = 0
    max_dis = 100.0
    max_ang = 5.0

    ft = [0.0,0.0,-10.0,0.0,0.0,0.0]

    # Spiral search parameters
    rcs = 0
    dr = 0.7
    fFinish = 1.0
    t = 60000.0
    vmax = 3.0

    # Linear insertion parameters
    force_goal = 20.0
    lin_v = 0.0
    lin_a = 0.0
    disMax = 100.0
    linorn = 1

    # Rotational insertion parameters
    angVelRot = 2.0
    forceInsertion = 1.0
    angleMax = 45
    orn = 1
    angAccmax = 0.0
    rotorn = 1

    # === Spiral Search ===
    select1 = [0, 0, 1, 1, 1, 0]
    robot.FT_Control(status, sensor_num, select1, ft, gain, adj_sign, ILC_sign, max_dis, max_ang, 0, 0, 0)
    rtn = robot.FT_SpiralSearch(rcs, dr, fFinish, t, vmax)
    print(f"FT_SpiralSearch rtn is {rtn}")
    robot.FT_Control(0, sensor_num, select1, ft, gain, adj_sign, ILC_sign, max_dis, max_ang, 0, 0, 0)

    # === Linear Insertion ===
    select2 = [1, 1, 1, 0, 0, 0]
    gain[0] = 0.00005
    ft[2] = -30.0
    robot.FT_Control(1, sensor_num, select2, ft, gain, adj_sign, ILC_sign, max_dis, max_ang, 0, 0, 0)
    rtn = robot.FT_LinInsertion(rcs, force_goal, lin_v, lin_a, disMax, linorn)
    print(f"FT_LinInsertion rtn is {rtn}")
    robot.FT_Control(0, sensor_num, select2, ft, gain, adj_sign, ILC_sign, max_dis, max_ang, 0, 0, 0)

    # === Rotational Insertion ===
    select3 = [0, 0, 1, 1, 1, 0]
    ft[2] = -10.0
    gain[0] = 0.0001
    robot.FT_Control(1, sensor_num, select3, ft, gain, adj_sign, ILC_sign, max_dis, max_ang, 0, 0, 0)
    rtn = robot.FT_RotInsertion(rcs, angVelRot, forceInsertion, angleMax, orn, angAccmax, rotorn)
    print(f"FT_RotInsertion rtn is {rtn}")
    robot.FT_Control(0, sensor_num, select3, ft, gain, adj_sign, ILC_sign, max_dis, max_ang, 0, 0, 0)

    # === Linear Insertion again ===
    select4 = [1, 1, 1, 0, 0, 0]
    ft[2] = -30.0
    robot.FT_Control(1, sensor_num, select4, ft, gain, adj_sign, ILC_sign, max_dis, max_ang, 0, 0, 0)
    rtn = robot.FT_LinInsertion(rcs, force_goal, lin_v, lin_a, disMax, linorn)
    print(f"FT_LinInsertion rtn is {rtn}")
    robot.FT_Control(0, sensor_num, select4, ft, gain, adj_sign, ILC_sign, max_dis, max_ang, 0, 0, 0)

    robot.CloseRPC()

def TestSurface(self):
    company = 24
    device = 0
    softversion = 0
    bus = 1
    index = 1

    robot.FT_SetConfig(company, device, softversion, bus)
    time.sleep(1)

    error,[company, device, softversion, bus] = robot.FT_GetConfig()
    print(f"FT config:{company},{device},{softversion},{bus}")
    time.sleep(1)

    robot.FT_Activate(0)
    time.sleep(1)
    robot.FT_Activate(1)
    time.sleep(1)

    robot.FT_SetZero(0)
    time.sleep(1)

    # Surface finding parameters
    rcs = 0
    dir = 1
    axis = 1
    lin_v = 3.0
    lin_a = 0.0
    maxdis = 50.0
    ft_goal = 2.0

    desc_pos = [-419.524, -13.000, 351.569, -178.118, 0.314, 3.833]
    xcenter = [0, 0, 0, 0, 0, 0]
    ycenter = [0, 0, 0, 0, 0, 0]

    ft = [-2.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    # Move to initial position
    robot.MoveCart(desc_pos, 9, 0, 100.0)

    # X-axis surface finding
    robot.FT_CalCenterStart()
    robot.FT_FindSurface(rcs, dir, axis, lin_v, lin_a, maxdis, ft_goal)
    robot.MoveCart(desc_pos, 9, 0)
    robot.WaitMs(1000)

    dir = 2
    robot.FT_FindSurface(rcs, dir, axis, lin_v, lin_a, maxdis, ft_goal)
    error,xcenter = robot.FT_CalCenterEnd()
    print(f"xcenter:{xcenter[0]},{xcenter[1]},{xcenter[2]},{xcenter[3]},{xcenter[4]},{xcenter[5]}")
    robot.MoveCart(xcenter, 9, 0, 60.0)

    # Y-axis surface finding
    robot.FT_CalCenterStart()
    dir = 1
    axis = 2
    lin_v = 6.0
    maxdis = 150.0
    robot.FT_FindSurface(rcs, dir, axis, lin_v, lin_a, maxdis, ft_goal)
    robot.MoveCart(desc_pos, 9, 0, 100.0)
    robot.WaitMs(1000)

    dir = 2
    robot.FT_FindSurface(rcs, dir, axis, lin_v, lin_a, maxdis, ft_goal)
    error,ycenter = robot.FT_CalCenterEnd()
    print(f"ycenter:{ycenter[0]},{ycenter[1]},{ycenter[2]},{ycenter[3]},{ycenter[4]},{ycenter[5]}")
    robot.MoveCart(ycenter, 9, 0, 60.0)

    robot.CloseRPC()

def TestCompliance(self):
    company = 24
    device = 0
    softversion = 0
    bus = 1
    index = 1

    robot.FT_SetConfig(company, device, softversion, bus)
    time.sleep(1)

    error,[company, device, softversion, bus] = robot.FT_GetConfig()
    print(f"FT config:{company},{device},{softversion},{bus}")
    time.sleep(1)

    robot.FT_Activate(0)
    time.sleep(1)
    robot.FT_Activate(1)
    time.sleep(1)

    robot.FT_SetZero(0)
    time.sleep(1)

    # FT control parameters
    flag = 1
    sensor_id = 1
    select = [1, 1, 1, 0, 0, 0]
    ft_pid = [0.0005, 0.0, 0.0, 0.0, 0.0, 0.0]
    adj_sign = 0
    ILC_sign = 0
    max_dis = 100.0
    max_ang = 0.0

    # ForceTorque values
    ft = [-10.0, -10.0, -10.0, 0.0, 0.0, 0.0]

    offset_pos = [0.0,0.0,0.0,0.0,0.0,0.0]  # 替代 DescPose(0, 0, 0, 0, 0, 0)
    epos = [0.0,0.0,0.0,0.0]  # 替代 ExaxisPos(0, 0, 0, 0)

    j1 = [-11.904, -99.669, 117.473, -108.616, -91.726, 74.256]  # JointPos
    j2 = [-45.615, -106.172, 124.296, -107.151, -91.282, 74.255]

    desc_p1 = [-419.524, -13.000, 351.569, -178.118, 0.314, 3.833]  # DescPose
    desc_p2 = [-321.222, 185.189, 335.520, -179.030, -1.284, -29.869]

    # Start FT control and compliance
    robot.FT_Control(flag, sensor_id, select, ft, ft_pid, adj_sign, ILC_sign, max_dis, max_ang, 0, 0, 0)

    p = 0.00005
    force = 30.0
    rtn = robot.FT_ComplianceStart(p, force)
    print(f"FT_ComplianceStart rtn is {rtn}")

    count = 3
    while count > 0:
        robot.MoveL(desc_pos=desc_p1,tool= 0,user= 0,vel= 100.0)
        robot.MoveL(desc_pos=desc_p2,tool= 0,user= 0,vel= 100.0)
        count -= 1

    robot.FT_ComplianceStop()
    print(f"FT_ComplianceStop rtn is {rtn}")

    flag = 0
    robot.FT_Control(flag, sensor_id, select, ft, ft_pid, adj_sign, ILC_sign, max_dis, max_ang, 0, 0, 0)

    robot.CloseRPC()


def TestEndForceDragCtrl(self):
    robot.SetForceSensorDragAutoFlag(1)

    M = [15.0, 15.0, 15.0, 0.5, 0.5, 0.1]
    B = [150.0, 150.0, 150.0, 5.0, 5.0, 1.0]
    K = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    F = [10.0, 10.0, 10.0, 1.0, 1.0, 1.0]

    robot.EndForceDragControl(1, 0, 0, 0, M, B, K, F, 50, 100)

    time.sleep(5)

    drag_state = 0
    six_dimensional_drag_state = 0
    error,drag_state, six_dimensional_drag_state = robot.GetForceAndTorqueDragState()
    print(f"the drag state is {drag_state} {six_dimensional_drag_state}")

    robot.EndForceDragControl(0, 0, 0, 0, M, B, K, F, 50, 100)
    robot.CloseRPC()


def TestForceAndJointImpedance(self):
    robot.DragTeachSwitch(1)

    lamde_dain = [3.0, 2.0, 2.0, 2.0, 2.0, 3.0]
    k_gain = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    b_gain = [150.0, 150.0, 150.0, 5.0, 5.0, 1.0]

    rtn = robot.ForceAndJointImpedanceStartStop(1, 0, lamde_dain, k_gain, b_gain, 1000, 180)
    print(f"ForceAndJointImpedanceStartStop rtn is {rtn}")

    time.sleep(5)

    robot.DragTeachSwitch(0)
    rtn = robot.ForceAndJointImpedanceStartStop(0, 0, lamde_dain, k_gain, b_gain, 1000, 180)
    print(f"ForceAndJointImpedanceStartStop rtn is {rtn}")

    robot.CloseRPC()


def TestUDPAxis(self):
    rtn = robot.ExtDevSetUDPComParam("192.168.58.88", 2021, 2, 100, 3, 200, 1, 100, 5,1)
    print(f"ExtDevSetUDPComParam rtn is {rtn}")

    ip = ""
    port = 0
    period = 0
    loss_pkg_time = 0
    loss_pkg_num = 0
    disconnect_time = 0
    reconnect_enable = 0
    reconnect_period = 0
    reconnect_num = 0

    rtn,[ip, port, period, loss_pkg_time, loss_pkg_num, disconnect_time, reconnect_enable, reconnect_period, reconnect_num] = robot.ExtDevGetUDPComParam()
    patam = (
        f"\nip {ip}\nport {port}\nperiod {period}\nlossPkgTime {loss_pkg_time}"
        f"\nlossPkgNum {loss_pkg_num}\ndisConntime {disconnect_time}"
        f"\nreconnecable {reconnect_enable}\nreconnperiod {reconnect_period}"
        f"\nreconnnun {reconnect_num}"
    )
    print(f"ExtDevGetUDPComParam rtn is {rtn}{patam}")

    robot.ExtDevLoadUDPDriver()

    rtn = robot.ExtAxisServoOn(1, 1)
    print(f"ExtAxisServoOn axis id 1 rtn is {rtn}")
    rtn = robot.ExtAxisServoOn(2, 1)
    print(f"ExtAxisServoOn axis id 2 rtn is {rtn}")

    time.sleep(2)

    robot.ExtAxisSetHoming(1, 0, 10, 2)
    time.sleep(2)
    rtn = robot.ExtAxisSetHoming(2, 0, 10, 2)
    print(f"ExtAxisSetHoming rtnn is {rtn}")

    time.sleep(4)

    rtn = robot.SetRobotPosToAxis(1)
    print(f"SetRobotPosToAxis rtn is {rtn}")

    rtn = robot.SetAxisDHParaConfig(10, 20, 0, 0, 0, 0, 0, 0, 0)
    print(f"SetAxisDHParaConfig rtn is {rtn}")

    rtn = robot.ExtAxisParamConfig(1, 1, 1, 1000, -1000, 1000, 1000, 1.905, 262144, 200, 1, 0, 0)
    print(f"ExtAxisParamConfig axis 1 rtn is {rtn}")

    rtn = robot.ExtAxisParamConfig(2, 1, 1, 1000, -1000, 1000, 1000, 4.444, 262144, 200, 1, 0, 0)
    print(f"ExtAxisParamConfig axis 1 rtn is {rtn}")

    time.sleep(3)
    robot.ExtAxisStartJog(1, 0, 10, 10, 30)
    time.sleep(1)
    robot.ExtAxisStopJog(1)
    time.sleep(3)
    robot.ExtAxisServoOn(1, 0)

    time.sleep(3)
    robot.ExtAxisStartJog(2, 0, 10, 10, 30)
    time.sleep(1)
    robot.ExtAxisStopJog(2)
    time.sleep(3)
    robot.ExtAxisServoOn(2, 0)

    robot.ExtDevUnloadUDPDriver()
    robot.CloseRPC()


def TestUDPAxisCalib(self):
    rtn = robot.ExtDevSetUDPComParam("192.168.58.88", 2021, 2, 100, 3, 200, 1, 100, 5, 1)
    print(f"ExtDevSetUDPComParam rtn is {rtn}")

    # udp_params = ["", 0, 0, 0, 0, 0, 0, 0, 0]
    rtn,udp_params = robot.ExtDevGetUDPComParam()
    ip, port, period, lossPkgTime, lossPkgNum, disconnectTime, reconnectEnable, reconnectPeriod, reconnectNum = udp_params
    patam = (
        f"\nip {ip}\nport {port}\nperiod {period}\nlossPkgTime {lossPkgTime}\n"
        f"lossPkgNum {lossPkgNum}\ndisConntime {disconnectTime}\nreconnecable {reconnectEnable}\n"
        f"reconnperiod {reconnectPeriod}\nreconnnun {reconnectNum}"
    )
    print(f"ExtDevGetUDPComParam rtn is {rtn}{patam}")

    robot.ExtDevLoadUDPDriver()

    rtn = robot.ExtAxisServoOn(1, 1)
    print(f"ExtAxisServoOn axis id 1 rtn is {rtn}")
    rtn = robot.ExtAxisServoOn(2, 1)
    print(f"ExtAxisServoOn axis id 2 rtn is {rtn}")
    time.sleep(2)

    robot.ExtAxisSetHoming(1, 0, 10, 2)
    time.sleep(2)
    rtn = robot.ExtAxisSetHoming(2, 0, 10, 2)
    print(f"ExtAxisSetHoming rtn is {rtn}")
    time.sleep(4)

    rtn = robot.SetRobotPosToAxis(1)
    print(f"SetRobotPosToAxis rtn is {rtn}")
    rtn = robot.SetAxisDHParaConfig(1, 128.5, 206.4, 0, 0, 0, 0, 0, 0)
    print(f"SetAxisDHParaConfig rtn is {rtn}")
    rtn = robot.ExtAxisParamConfig(1, 1, 1, 1000, -1000, 1000, 1000, 1.905, 262144, 200, 1, 0, 0)
    print(f"ExtAxisParamConfig axis 1 rtn is {rtn}")
    rtn = robot.ExtAxisParamConfig(2, 1, 1, 1000, -1000, 1000, 1000, 4.444, 262144, 200, 1, 0, 0)
    print(f"ExtAxisParamConfig axis 2 rtn is {rtn}")

    toolCoord = [0, 0, 210, 0, 0, 0]
    robot.SetToolCoord(1, toolCoord, 0, 0, 1, 0)

    jSafe = [115.193, -96.149, 92.489, -87.068, -89.15, -83.488]
    j1 = [117.559, -92.624, 100.329, -96.909, -94.057, -83.488]
    j2 = [112.239, -90.096, 99.282, -95.909, -89.824, -83.488]
    j3 = [110.839, -83.473, 93.166, -89.22, -90.499, -83.487]
    j4 = [107.935, -83.572, 95.424, -92.873, -87.933, -83.488]

    descSafe = [0.0,0.0,0.0,0.0,0.0,0.0]
    desc1 = [0.0,0.0,0.0,0.0,0.0,0.0]
    desc2 = [0.0,0.0,0.0,0.0,0.0,0.0]
    desc3 = [0.0,0.0,0.0,0.0,0.0,0.0]
    desc4 = [0.0,0.0,0.0,0.0,0.0,0.0]
    exaxisPos = [0.0,0.0,0.0,0.0]
    offdese = [0.0,0.0,0.0,0.0,0.0,0.0]

    error, descSafe = robot.GetForwardKin(jSafe)
    robot.MoveJ(joint_pos=jSafe,tool= 1,user= 0,vel= 100)
    time.sleep(2)

    error, desc1 = robot.GetForwardKin(j1)
    robot.MoveJ(joint_pos=j1,tool= 1,user= 0,vel= 100)
    time.sleep(2)

    actualTCPPos = [0.0,0.0,0.0,0.0,0.0,0.0]
    error, actualTCPPos = robot.GetActualTCPPose(0)
    robot.SetRefPointInExAxisEnd(actualTCPPos)
    rtn = robot.PositionorSetRefPoint(1)
    print(f"PositionorSetRefPoint 1 rtn is {rtn}")
    time.sleep(2)

    robot.MoveJ(joint_pos=jSafe,tool= 1,user= 0,vel= 100)
    robot.ExtAxisStartJog(1, 0, 50, 50, 10)
    time.sleep(1)
    robot.ExtAxisStartJog(2, 0, 50, 50, 10)
    time.sleep(1)
    error, desc2 = robot.GetForwardKin(j2)
    rtn = robot.MoveJ(joint_pos=j2,tool= 1,user= 0,vel= 100)
    rtn = robot.PositionorSetRefPoint(2)
    print(f"PositionorSetRefPoint 2 rtn is {rtn}")
    time.sleep(2)

    robot.MoveJ(joint_pos=jSafe,tool= 1,user= 0,vel= 100)
    robot.ExtAxisStartJog(1, 0, 50, 50, 10)
    time.sleep(1)
    robot.ExtAxisStartJog(2, 0, 50, 50, 10)
    time.sleep(1)
    error, desc3 = robot.GetForwardKin(j3)
    robot.MoveJ(joint_pos=j3,tool= 1,user= 0,vel= 100)
    rtn = robot.PositionorSetRefPoint(3)
    print(f"PositionorSetRefPoint 3 rtn is {rtn}")
    time.sleep(2)

    robot.MoveJ(joint_pos=jSafe,tool= 1,user= 0,vel= 100)
    robot.ExtAxisStartJog(1, 0, 50, 50, 10)
    time.sleep(1)
    robot.ExtAxisStartJog(2, 0, 50, 50, 10)
    time.sleep(1)
    error, desc4 = robot.GetForwardKin(j4)
    robot.MoveJ(joint_pos=j4,tool= 1,user= 0,vel= 100)
    rtn = robot.PositionorSetRefPoint(4)
    print(f"PositionorSetRefPoint 4 rtn is {rtn}")
    time.sleep(2)

    axisCoord = [0.0,0.0,0.0,0.0,0.0,0.0]
    error,axisCoord = robot.PositionorComputeECoordSys()
    robot.MoveJ(joint_pos=jSafe,tool= 1,user= 0,vel= 100)
    print(f"PositionorComputeECoordSys rtn is {axisCoord[0]} {axisCoord[1]} {axisCoord[2]} {axisCoord[3]} {axisCoord[4]} {axisCoord[5]}")
    rtn = robot.ExtAxisActiveECoordSys(3, 1, axisCoord, 1)
    print(f"ExtAxisActiveECoordSys rtn is {rtn}")

    robot.CloseRPC()


def TestAuxDOAO(self):
    # 打开全部 AuxDO
    for i in range(128):
        robot.SetAuxDO(i, True, False, True)
        time.sleep(0.1)

    # 关闭全部 AuxDO
    for i in range(128):
        robot.SetAuxDO(i, False, False, True)
        time.sleep(0.1)

    # 设置 AuxAO 通道
    for i in range(409):
        value1 = i * 10
        value2 = 4095 - i * 10
        robot.SetAuxAO(0, value1, True)
        robot.SetAuxAO(1, value2, True)
        robot.SetAuxAO(2, value1, True)
        robot.SetAuxAO(3, value2, True)
        time.sleep(0.01)

    # 设置滤波时间
    robot.SetAuxDIFilterTime(10)
    robot.SetAuxAIFilterTime(0, 10)

    # 读取 AuxDI
    for i in range(20):
        curValue = False
        error, curValue = robot.GetAuxDI(i, False)  # 注意：如库内部需引用方式，这里需修改
        print(f"DI{i}   {curValue}")

    # 读取 AuxAI
    curValue = -1
    for i in range(4):
        error, curValue = robot.GetAuxAI(i, True)  # 同样注意引用传参问题
        print(f"AI{i}   {curValue}")

    # 等待 DI/AI
    robot.WaitAuxDI(1, False, 1000, False)
    robot.WaitAuxAI(1, 1, 132, 1000, False)

    robot.CloseRPC()


def TestTractor(self):
    robot.ExtDevSetUDPComParam("192.168.58.2", 2021, 2, 50, 5, 50, 1, 50, 10, 1)
    robot.ExtDevLoadUDPDriver()

    rtn = robot.ExtAxisServoOn(1, 1)
    rtn = robot.ExtAxisServoOn(2, 1)
    time.sleep(2)

    robot.ExtAxisSetHoming(1, 0, 10, 2)
    time.sleep(2)
    rtn = robot.ExtAxisSetHoming(2, 0, 10, 2)
    time.sleep(4)

    robot.ExtAxisParamConfig(1, 0, 0, 50000, -50000, 1000, 1000, 6.280, 16384, 200, 0, 0, 0)
    robot.ExtAxisParamConfig(2, 0, 0, 50000, -50000, 1000, 1000, 6.280, 16384, 200, 0, 0, 0)
    robot.SetAxisDHParaConfig(5, 0, 0, 0, 0, 0, 0, 0, 0)

    robot.TractorEnable(False)
    time.sleep(2)
    robot.TractorEnable(True)
    time.sleep(2)
    robot.TractorHoming()
    time.sleep(2)

    robot.TractorMoveL(100, 2)
    time.sleep(5)
    robot.TractorStop()

    robot.TractorMoveL(-100, 20)
    time.sleep(5)

    robot.TractorMoveC(300, 90, 20)
    time.sleep(10)

    robot.TractorMoveC(300, -90, 20)
    time.sleep(1)

    robot.CloseRPC()

def TestFIR(self):
    startjointPos = [-11.904, -99.669, 117.473, -108.616, -91.726, 74.256]
    midjointPos = [-45.615, -106.172, 124.296, -107.151, -91.282, 74.255]
    endjointPos = [-29.777, -84.536, 109.275, -114.075, -86.655, 74.257]

    startdescPose = [-419.524, -13.000, 351.569, -178.118, 0.314, 3.833]
    middescPose = [-321.222, 185.189, 335.520, -179.030, -1.284, -29.869]
    enddescPose = [-487.434, 154.362, 308.576, 176.600, 0.268, -14.061]

    # startjointPos = [-38.624,-78.619,90.307,-95.916,-94.168,73.744]
    # midjointPos = [-63.314,-96.329,90.374,-81.129,-93.465,73.662]
    # endjointPos = [-99.647,-90.279,106.709,-102.546,-91.529,73.687]
    #
    # startdescPose = [-498.294,276.877,379.142,-177.615,-6.707,-22.297]
    # middescPose = [-284.598,352.539,510.538,-177.494,-3.768,-46.969]
    # enddescPose = [-18.866,481.758,358.634,-179.624,-4.157,-83.296]

    exaxisPos = [0.0, 0.0, 0.0, 0.0]
    offdese = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    # rtn = robot.PtpFIRPlanningStart(200.0, 1000.0)
    rtn = robot.PtpFIRPlanningStart(1000.0, 1000.0)
    print(f"PtpFIRPlanningStart rtn is {rtn}")
    error = robot.MoveJ(joint_pos=startjointPos,tool= 0,user= 0,desc_pos=startdescPose,vel= 100,acc=100,ovl=100, blendT=-1.0, offset_flag=0)
    print(f"MoveJ rtn is {rtn}")
    error = robot.MoveJ(joint_pos=endjointPos,tool= 0,user= 0,desc_pos=enddescPose,vel= 100,acc=100,ovl=100, blendT=-1.0, offset_flag=0)
    print(f"MoveJ rtn is {rtn}")
    robot.PtpFIRPlanningEnd()
    print(f"PtpFIRPlanningEnd rtn is {rtn}")

    # rtn = robot.LinArcFIRPlanningStart(2000, 10000, 720, 1440)
    rtn = robot.LinArcFIRPlanningStart(1000, 1000, 1000, 1000)
    print(f"LinArcFIRPlanningStart rtn is {rtn}")
    error = robot.MoveL(desc_pos=startdescPose,tool= 0,user= 0, joint_pos=startjointPos,vel= 100,overSpeedStrategy=1,speedPercent=1)
    print(f"MoveL rtn is {rtn}")
    error = robot.MoveC(desc_pos_p=middescPose,tool_p= 0,user_p= 0, joint_pos_p=midjointPos,vel_p= 100,desc_pos_t=enddescPose,tool_t= 0,user_t= 0,joint_pos_t=endjointPos,vel_t= 100)
    print(f"MoveC rtn is {rtn}")
    robot.LinArcFIRPlanningEnd()
    print(f"LinArcFIRPlanningEnd rtn is {rtn}")

    robot.CloseRPC()

def TestAccSmooth(self):
    startjointPos = [-11.904, -99.669, 117.473, -108.616, -91.726, 74.256]
    endjointPos = [-45.615, -106.172, 124.296, -107.151, -91.282, 74.255]

    startdescPose = [-419.524, -13.000, 351.569, -178.118, 0.314, 3.833]
    enddescPose = [-321.222, 185.189, 335.520, -179.030, -1.284, -29.869]

    exaxisPos = [0.0, 0.0, 0.0, 0.0]
    offdese = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    rtn = robot.AccSmoothStart(0)
    print(f"AccSmoothStart rtn is {rtn}")
    robot.MoveJ(joint_pos=startjointPos,tool= 0,user= 0,vel= 100)
    robot.MoveJ(joint_pos=endjointPos,tool= 0,user= 0,vel= 100)
    rtn = robot.AccSmoothEnd(0)
    print(f"AccSmoothEnd rtn is {rtn}")

    robot.CloseRPC()


def TestAngularSpeed(self):
    startjointPos = [-11.904, -99.669, 117.473, -108.616, -91.726, 74.256]
    endjointPos = [-45.615, -106.172, 124.296, -107.151, -91.282, 74.255]

    startdescPose = [-419.524, -13.000, 351.569, -178.118, 0.314, 3.833]
    enddescPose = [-321.222, 185.189, 335.520, -179.030, -1.284, -29.869]

    exaxisPos = [0.0, 0.0, 0.0, 0.0]
    offdese = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    rtn = robot.AngularSpeedStart(50)
    print(f"AngularSpeedStart rtn is {rtn}")
    robot.MoveJ(joint_pos=startjointPos, tool=0,user= 0,vel= 100)
    robot.MoveJ(joint_pos=endjointPos, tool=0,user= 0,vel= 100)
    rtn = robot.AngularSpeedEnd()
    print(f"AngularSpeedEnd rtn is {rtn}")

    robot.CloseRPC()



def TestSingularAvoid(self):
    startjointPos = [-11.904, -99.669, 117.473, -108.616, -91.726, 74.256]
    endjointPos = [-45.615, -106.172, 124.296, -107.151, -91.282, 74.255]

    startdescPose = [-419.524, -13.000, 351.569, -178.118, 0.314, 3.833]
    enddescPose = [-321.222, 185.189, 335.520, -179.030, -1.284, -29.869]

    exaxisPos = [0.0, 0.0, 0.0, 0.0]
    offdese = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    rtn = robot.SingularAvoidStart(2, 10, 5, 5)
    print(f"SingularAvoidStart rtn is {rtn}")
    robot.MoveJ(joint_pos=startjointPos, tool=0,user= 0,vel= 100)
    robot.MoveJ(joint_pos=endjointPos, tool=0,user= 0,vel= 100)
    rtn = robot.SingularAvoidEnd()
    print(f"SingularAvoidEnd rtn is {rtn}")

    robot.CloseRPC()


def TestLoadTrajLA(self):
    rtn = robot.TrajectoryJUpLoad("D://zUP/traj.txt")
    print(f"Upload TrajectoryJ A {rtn}")

    traj_file_name = "/fruser/traj/traj.txt"
    rtn = robot.LoadTrajectoryLA(traj_file_name, 1, 2, 0, 2, 50, 200, 1000)
    print(f"LoadTrajectoryLA {traj_file_name}, rtn is: {rtn}")

    rtn, traj_start_pose = robot.GetTrajectoryStartPose(traj_file_name)
    print(f"GetTrajectoryStartPose is: {rtn}")
    print(f"desc_pos: {traj_start_pose[0]},{traj_start_pose[1]},{traj_start_pose[2]},{traj_start_pose[3]},{traj_start_pose[4]},{traj_start_pose[5]}")

    time.sleep(1)

    robot.SetSpeed(50)
    robot.MoveCart(traj_start_pose, 0, 0, 100, 100, 100)

    rtn = robot.MoveTrajectoryLA()
    print(f"MoveTrajectoryLA rtn is: {rtn}")

    robot.CloseRPC()

def TestIdentify(self):
    retval = robot.LoadIdentifyDynFilterInit()
    print(f"LoadIdentifyDynFilterInit retval is: {retval}")

    retval = robot.LoadIdentifyDynVarInit()
    print(f"LoadIdentifyDynVarInit retval is: {retval}")

    # posJ = [0.0] * 6
    # posDec = [0.0] * 6
    # joint_toq = [0.0] * 6

    error, posJ = robot.GetActualJointPosDegree(0)
    posJ[1] += 10  # Modify joint 2

    error, joint_toq = robot.GetJointTorques(0)
    joint_toq[1] += 2  # Modify torque on joint 2

    tmpTorque = joint_toq.copy()

    retval = robot.LoadIdentifyMain(tmpTorque, posJ, 1)
    print(f"LoadIdentifyMain retval is: {retval}")

    gain = [0, 0.05, 0, 0, 0, 0, 0, 0.02, 0, 0, 0, 0]
    weight = [0.0]
    load_pos = [0.0, 0.0, 0.0]

    retval, weight, load_pos = robot.LoadIdentifyGetResult(gain)
    print(f"LoadIdentifyGetResult retval is: {retval} ; weight is {weight}  cog is {load_pos[0]} {load_pos[1]} {load_pos[2]}")

    robot.CloseRPC()

def TestUDPAxisCalib():
    axisPos = [20,0,0,0]
    robot.ExtAxisMove(axisPos, 50)
    robot.CloseRPC()
    return 0


def testSyncMoveJ():
    # 设置UDP通信参数并加载
    robot.ExtDevSetUDPComParam("192.168.58.88", 2021, 2, 100, 3, 100, 1, 100, 10)
    robot.ExtDevLoadUDPDriver()

    # 设置扩展轴参数
    robot.SetAxisDHParaConfig(4, 200, 200, 0, 0, 0, 0, 0, 0)
    robot.SetRobotPosToAxis(1)
    robot.ExtAxisParamConfig(1, 0, 1, 100, -100, 10, 10, 12, 131072, 0, 1, 0, 0)

    # 扩展轴使能、回零
    robot.ExtAxisServoOn(1, 0)
    robot.ExtAxisSetHoming(1, 0, 20, 3)

    # 扩展轴坐标系标定
    pos = []  # 请在此填写标定点坐标
    robot.SetRefPointInExAxisEnd(pos)
    robot.PositionorSetRefPoint(1)  # 此操作应重复4次（用4个点）

    error,coord = robot.PositionorComputeECoordSys()
    robot.ExtAxisActiveECoordSys(1, 1, coord, 1)

    # 同步运动起点与终点
    startdescPose = []  # 请填写具体坐标
    startjointPos = []  # 请填写具体坐标
    startexaxisPos = []  # 请填写具体坐标

    enddescPose = []  # 请填写具体坐标
    endjointPos = []  # 请填写具体坐标
    endexaxisPos = []  # 请填写具体坐标

    # 运动到起始点
    robot.ExtAxisMove(startexaxisPos, 20)
    offdese = [0, 0, 0, 0, 0, 0]
    robot.MoveJ(joint_pos=startjointPos,tool= 1,user= 1,vel= 100,acc= 100,ovl= 100,exaxis_pos= startexaxisPos,blendT= 0,offset_flag= 0,offset_pos= offdese)

    # 执行同步运动
    robot.ExtAxisSyncMoveJ(endjointPos, enddescPose, 1, 1, endexaxisPos, 100, 100, 100, -1, 0, offdese)

    robot.CloseRPC()

def testSyncMoveL():
    # 设置UDP通信参数并加载
    robot.ExtDevSetUDPComParam("192.168.58.88", 2021, 2, 100, 3, 100, 1, 100, 10)
    robot.ExtDevLoadUDPDriver()

    # 设置扩展轴参数
    robot.SetAxisDHParaConfig(4, 200, 200, 0, 0, 0, 0, 0, 0)
    robot.SetRobotPosToAxis(1)
    robot.ExtAxisParamConfig(1, 0, 1, 100, -100, 10, 10, 12, 131072, 0, 1, 0, 0)

    # 扩展轴使能、回零
    robot.ExtAxisServoOn(1, 0)
    robot.ExtAxisSetHoming(1, 0, 20, 3)

    # 扩展轴坐标系标定
    pos = []  # 请填写标定点坐标
    robot.SetRefPointInExAxisEnd(pos)
    robot.PositionorSetRefPoint(1)  # 需调用4次用于标定
    error,coord = robot.PositionorComputeECoordSys()
    robot.ExtAxisActiveECoordSys(1, 1, coord, 1)

    # 同步运动起点与终点
    startdescPose = []  # 填写坐标
    startjointPos = []  # 填写坐标
    startexaxisPos = []  # 填写坐标

    enddescPose = []  # 填写坐标
    endjointPos = []  # 填写坐标
    endexaxisPos = []  # 填写坐标

    # 运动到起始点
    robot.ExtAxisMove(startexaxisPos, 20)
    offdese = [0, 0, 0, 0, 0, 0]
    robot.MoveJ(joint_pos=startjointPos, tool= 1,user= 1,vel= 100,acc= 100,ovl= 100,exaxis_pos= startexaxisPos,blendT= 0)

    # 执行同步直线运动
    robot.ExtAxisSyncMoveL(endjointPos, enddescPose, 1, 1, endexaxisPos, 100, 100, 100, 0, 0, offdese)

    robot.CloseRPC()

def testSyncMoveC():
    # 设置UDP通信参数并加载
    robot.ExtDevSetUDPComParam("192.168.58.88", 2021, 2, 100, 3, 100, 1, 100, 10)
    robot.ExtDevLoadUDPDriver()

    # 设置扩展轴参数
    robot.SetAxisDHParaConfig(4, 200, 200, 0, 0, 0, 0, 0, 0)
    robot.SetRobotPosToAxis(1)
    robot.ExtAxisParamConfig(1, 0, 1, 100, -100, 10, 10, 12, 131072, 0, 1, 0, 0)

    # 扩展轴使能、回零
    robot.ExtAxisServoOn(1, 0)
    robot.ExtAxisSetHoming(1, 0, 20, 3)

    # 扩展轴坐标系标定
    pos = []  # 输入标定点坐标
    robot.SetRefPointInExAxisEnd(pos)
    robot.PositionorSetRefPoint(1)  # 调用4次以完成标定
    coord = []
    error,coord = robot.PositionorComputeECoordSys()
    robot.ExtAxisActiveECoordSys(1, 1, coord, 1)

    # 同步圆弧起始点、中间点、终点
    startdescPose = []# 输入坐标
    startjointPos = []# 输入坐标
    startexaxisPos =[]  # 输入扩展轴坐标

    middescPose = []# 输入中间点
    midjointPos = []
    midexaxisPos =[]

    enddescPose = []
    endjointPos = []
    endexaxisPos =[]

    # 运动到起始点
    robot.ExtAxisMove(startexaxisPos, 20)
    offdese = [0, 0, 0, 0, 0, 0]
    robot.MoveJ(joint_pos=startjointPos,tool= 1,user= 1,vel= 100,acc= 100,ovl= 100,exaxis_pos= startexaxisPos,blendT= 0,offset_flag= 0,offset_pos= offdese)

    # 开始同步圆弧运动
    robot.ExtAxisSyncMoveC(midjointPos,middescPose,1,1,midexaxisPos,
                           endjointPos,enddescPose,1,1,endexaxisPos,
                           100,100,0,offdese,
                           100,100,0,offdese,
                           100,0)
    robot.CloseRPC()







###6.9
# TestRobotCtrl(robot)
# TestGetVersions(robot)
# TestJOG(robot)
# TestMove(robot)
# TestSpiral(robot)
# TestServoJ(robot)
# TestServoCart(robot)
# TestSpline(robot)
# TestNewSpline(robot)
# TestPause(robot)
# TestOffset(robot)
# TestMoveAO(robot)
# TestAODO(robot)
# TestGetDIAI(robot)
# TestWaitDIAI(robot)
# TestDOReset(robot)
# TestTCPCompute(robot)
# TestWobjCoord(robot)
# TestExtCoord(robot)
######946行
# TestLoadInstall(robot)
# TestFriction(robot)
# TestGetError(robot)
# TestCollision(robot)
######1134行



###6.10
# TestLimit(robot)
# TestCollisionMethod(robot)
# TestPowerLimit(robot)
# TestServoJT(robot)
# TestGetStatus(robot)
# TestInverseKin(robot)
# TestTPD(robot)
# TestGetTeachPoint(robot)
# TestTraj(robot)
# TestLuaOp(robot)
# TestLUAUpDownLoad(robot)
# TestGripper(robot)
# TestRotGripperState(robot)
TestConveyor(robot)################################################未完成
# TestAxleSensor(robot)
# TestExDevProtocol(robot)
# TestAxleLua(robot)
# TestSetWeldParam(robot)
# TestWelding(robot)
# TestSegWeld(robot)
# TestWeave(robot)
# TestSSHMd5(robot)
# TestRealtimePeriod(robot)
# TestUpgrade(robot)
# TestPointTable(robot)
###############2314



# TestDownLoadRobotData(robot)
# TestExtDIConfig(robot)
# TestArcWeldTrace(robot)
#############2515



# TestWireSearch(robot)
# TestFTInit(robot)
# TestFTLoadCompute(robot)
# TestFTGuard(robot)
# TestFTControl(robot)
# TestFTSearch(robot)
# TestSurface(robot)
# TestCompliance(robot)
# TestEndForceDragCtrl(robot)
# TestForceAndJointImpedance(robot)
# TestUDPAxis(robot)
# TestUDPAxisCalib(robot)
# TestAuxDOAO(robot)
# TestTractor(robot)
# TestFIR(robot)
# TestAccSmooth(robot)
# TestAngularSpeed(robot)
# TestSingularAvoid(robot)
# TestLoadTrajLA(robot)
# TestIdentify(robot)
# (robot)
# (robot)
# (robot)
# (robot)
# Test485Auxservo(robot)
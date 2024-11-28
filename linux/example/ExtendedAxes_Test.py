from fairino import Robot
import time

# 与机器人控制器建立连接，连接成功返回一个机器人对象
robot = Robot.RPC('192.168.58.2')


def SDK_InitializeCoordinates():  # 将所有坐标修改为默认0号坐标
    robot.SetToolList(0, [0, 0, 0, 0, 0, 0], 0, 0)
    robot.SetWObjList(0, [0, 0, 0, 0, 0, 0])
    robot.SetExToolCoord(0, [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0])


def SDK_UDP_SetExtAxis(axisCoordNum=0, toolNum=0, coord=None, calibFlag=0):
    """
    axisCoordNum：扩展轴号，采用二进制编码；

    toolNum：坐标系编号；

    coord：坐标系值[x,y,z,rx,ry,rz]；

    calibFlag：标定标志 0-否，1-是；

    """
    if coord is None:
        coord = [0, 0, 0, 0, 0, 0]
    error = robot.ExtAxisActiveECoordSys(axisCoordNum, toolNum, coord, calibFlag)
    assert error == 0, '设置扩展轴坐标失败，请检查SDK逻辑！'
    print('当前已启用扩展轴：{}'.format(toolNum) + '\n错误码为：{}'.format(error) + '\n当前扩展轴坐标值为：{}'.format(coord))


def SDK_UDP_LinkExAxis():  # 通过UDP连接进行通讯
    """
    ip：PLC IP地址；

    port：端口号；

    period：通讯周期(ms，暂不开放)；

    lossPkgTime：丢包检测时间(ms)；

    lossPkgNum：丢包次数；

    disconnectTime：通讯断开确认时长；

    reconnectEnable：通讯断开自动重连使能 0-不使能 1-使能；

    reconnectPeriod：重连周期间隔(ms)；

    reconnectNum：重连次数

    :return:
            错误码 成功-0 失败- errcode
    """
    # 初始化坐标系
    # SDK_InitializeCoordinates()

    # 修改工具坐标系为对应测试工具
    error = robot.SetToolCoord(1, [0, 0, 225, 0, 0, 0], 0, 0)
    print('当前工具变更为：工具1，长度为：210,错误码为：{}'.format(error))

    # 配置UDP通讯参数
    error = robot.ExtDevSetUDPComParam("192.168.58.88", 2021, 2, 50, 5, 50, 1, 100, 10)
    print("ExtDevSetUDPComParam return:", error)
    assert error == 0, '请检查配置参数，或SDK异常！！'

    # 加载UDP通信
    error = robot.ExtDevLoadUDPDriver()
    print("ExtDevLoadUDPDriver return:", error)
    assert error == 0, '请检查是否正确配置对应参数，或SDK异常！！'


def SDK_UDP_Disconnect():  # 卸载UDP通信
    error = robot.ExtDevUnloadUDPDriver()
    print("ExtDevUnloadUDPDriver return:", error)
    assert error == 0, '请检查是否未加载，或SDK异常！！'


def SDK_UDP_Servo2():
    """
    方法名：ExtAxisServoOn()

    axisID：轴号[1-4]；

    status：0-去使能；1-使能；

    方法名：ExtAxisSetHoming()

    axisID：轴号[1-4]；

    mode：回零方式 0当前位置回零，1负限位回零，2-正限位回零

    searchVel 寻零速度(mm/s)；

    latchVel：寻零箍位速度(mm/s)；
    """
    # 伺服使能与去使能
    ext = (1, 2)
    for ext_num in ext:
        error = robot.ExtAxisServoOn(ext_num, 0)
        assert error == 0, '去使能失败'
        time.sleep(2)
        error = robot.ExtAxisServoOn(ext_num, 1)
        assert error == 0, '使能失败'
        time.sleep(2)
        print('轴{}使能已下发！'.format(ext_num))

        # 伺服回零
        error = robot.ExtAxisSetHoming(ext_num, 0, 5, 1)
        print(error)
        assert error == 0, '回零失败'


def SDK_UDP_ExtAxisJog(axisID=1, direction=1, vel=50, acc=50, maxDistance=10):
    """
    方法名：ExtAxisStartJog(axisID, direction, vel, acc, maxDistance)

    axisID：轴号[1-4]；

    direction：转动方向 0-反向；1-正向；

    vel：速度(mm/s)；

    acc：加速度(mm/s)；

    maxDistance：最大点动距离；

    方法名2：ExtAxisStopJog(axisID)

    axisID：轴号[1-4]；

    """
    assert maxDistance <= 10, '当前转动距离大于10，处于危险值,请修改至小于10以内重新开始！'

    # UDP扩展轴点动开始
    error = robot.ExtAxisStartJog(axisID, direction, vel, acc, maxDistance)
    print("ExtAxisStartJog return:", error)
    time.sleep(1)
    # UDP扩展轴点动停止
    error = robot.ExtAxisStopJog(axisID)
    print("ExtAxisStopJog return:", error)


def SDK_UDP_Calibration():
    """
SetAxisDHParaConfig(axisConfig,axisDHd1,axisDHd2,axisDHd3,axisDHd4,axisDHa1, axisDHa2,axisDHa3,axisDHa4)

    axisConfig：外部轴构型，0-单自由度直线滑轨，1-两自由度L型变位机，2-三自由度，3-四自由度，4-单自由度变位机；

    axisDHd1：外部轴DH参数d1 mm；

    axisDHd2：外部轴DH参数d2 mm；

    axisDHd3：外部轴DH参数d3 mm；

    axisDHd4：外部轴DH参数d4 mm；

    axisDHa1：外部轴DH参数a1 mm；

    axisDHa2：外部轴DH参数a2 mm；

    axisDHa3：外部轴DH参数a3 mm；

    axisDHa4：外部轴DH参数a4 mm；

SetAxisDHParaConfig(axisConfig,axisDHd1,axisDHd2,axisDHd3,axisDHd4,axisDHa1, axisDHa2,axisDHa3,axisDHa4)

    axisId：轴号[1-4]；

    axisType：扩展轴类型 0-平移；1-旋转；

    axisDirection：扩展轴方向 0-正向；1-反向；

    axisMax：扩展轴最大位置 mm；

    axisMin：扩展轴最小位置 mm；

    axisVel：速度mm/s；

    axisAcc：加速度mm/s2；

    axisLead：导程mm；

    encResolution：编码器分辨率；

    axisOffect：焊缝起始点扩展轴偏移量；

    axisCompany：驱动器厂家 0-禾川；0-汇川；0-松下；

    axisModel：动器型号 0-禾川-SV-XD3EA040L-E，1-禾川-SV-X2EA150A-A，0-汇川-SV620PT5R4I，0-松下-MADLN15SG，1-松下-MSDLN25SG，2-松下-MCDLN35SG；

    axisEncType：编码器类型 0-增量；1-绝对值；

ExtAxisActiveECoordSys(axisCoordNum,toolNum,coord,calibFlag)

    axisCoordNum：坐标系编号；

    toolNum：工具号；

    coord：坐标系值[x,y,z,rx,ry,rz]；

    calibFlag：标定标志 0-否，1-是；
    """
    # 设置扩展机器人相对扩展轴位置
    error = robot.SetRobotPosToAxis(1)
    print("设置扩展机器人相对扩展轴位置 return:", error)

    # 设置扩展轴系统DH参数配置
    error = robot.SetAxisDHParaConfig(1, 128.5, 206.4, 0, 0, 0, 0, 0, 0, )
    print("设置扩展轴系统DH参数配置 return:", error)

    # UDP扩展轴参数配置
    error = robot.ExtAxisParamConfig(1, 1, 0, 1000, -1000, 1000, 1000, 1.905, 262144, 200, 0, 0, 0)
    print("UDP扩展轴1参数配置 return:", error)
    error = robot.ExtAxisParamConfig(2, 1, 0, 1000, -1000, 1000, 1000, 4.444, 262144, 200, 0, 0, 0)
    print("UDP扩展轴2参数配置 return:", error)


def SDK_UDP_CalibrationPoint4():
    """
    标定参考点，4点法计算与应用
    """

    # 加载UPD通讯
    SDK_UDP_LinkExAxis()

    # 伺服初始化
    SDK_UDP_Servo2()

    # 设置标定参数：DH、扩展轴参数、扩展轴位置
    SDK_UDP_Calibration()

    # 进入安全点
    robot.MoveJ(joint_pos=[115.193, -96.149, 92.489, -87.068, -89.15, -83.488], tool=1, user=0)
    time.sleep(5)

    # 获取当前TCP位置信息，方便传入下方作为参考点
    robot.MoveJ(joint_pos=[117.559, -92.624, 100.329, -96.909, -94.057, -83.488], tool=1, user=0)  # 参考点
    time.sleep(5)
    error, desc_pos = robot.GetActualTCPPose()
    print("获取当前TCP位置信息为：", error, desc_pos)

    # 设置变位机的标定参考点
    error = robot.SetRefPointInExAxisEnd(desc_pos)
    print("标定参考点 return:", error)
    assert error == 0, '请重新标定参考点,或检查SDK逻辑！'

    # 变位机坐标系参考点设置-四点法
    error = robot.PositionorSetRefPoint(1)
    print("PositionorSetRefPoint(1) return:", error)
    time.sleep(2)
    # 进入安全点
    robot.MoveJ(joint_pos=[115.193, -96.149, 92.489, -87.068, -89.15, -83.488], tool=1, user=0)
    SDK_UDP_ExtAxisJog(1, 0, 50, 50, 10)  # 1轴，反转，速度50，加速度50，转动10mm
    SDK_UDP_ExtAxisJog(2, 0, 50, 50, 10)  # 2轴，反转，速度50，加速度50，转动10mm
    robot.MoveJ(joint_pos=[112.239, -90.096, 99.282, -95.909, -89.824, -83.488], tool=1, user=0)  # 点位2
    error = robot.PositionorSetRefPoint(2)
    print("PositionorSetRefPoint(2) return:", error)
    time.sleep(2)
    # 进入安全点
    robot.MoveJ(joint_pos=[115.193, -96.149, 92.489, -87.068, -89.15, -83.488], tool=1, user=0)
    SDK_UDP_ExtAxisJog(1, 0, 50, 50, 10)  # 1轴，反转，速度50，加速度50，转动10mm
    SDK_UDP_ExtAxisJog(2, 0, 50, 50, 10)  # 2轴，反转，速度50，加速度50，转动10mm
    robot.MoveJ(joint_pos=[110.839, -83.473, 93.166, -89.22, -90.499, -83.487], tool=1, user=0)  # 点位3
    error = robot.PositionorSetRefPoint(3)
    print("PositionorSetRefPoint(3) return:", error)
    time.sleep(2)
    # 进入安全点
    robot.MoveJ(joint_pos=[115.193, -96.149, 92.489, -87.068, -89.15, -83.488], tool=1, user=0)
    SDK_UDP_ExtAxisJog(1, 0, 50, 50, 10)  # 1轴，反转，速度50，加速度50，转动10mm
    SDK_UDP_ExtAxisJog(2, 0, 50, 50, 10)  # 2轴，反转，速度50，加速度50，转动10mm
    robot.MoveJ(joint_pos=[107.935, -83.572, 95.424, -92.873, -87.933, -83.488], tool=1, user=0)  # 点位4
    time.sleep(2)
    error = robot.PositionorSetRefPoint(4)
    print("PositionorSetRefPoint(4) return:", error)
    # 变位机坐标系计算-四点法
    error = robot.PositionorComputeECoordSys()
    print("PositionorComputeECoordSys() return:", error)
    assert error[0] == 0, '计算失败，请重新尝试'

    # 进入安全点
    robot.MoveJ(joint_pos=[115.193, -96.149, 92.489, -87.068, -89.15, -83.488], tool=1, user=0)
    SDK_UDP_ExtAxisJog(1, 1, 50, 50, 10)  # 1轴，正转，速度50，加速度50，转动10mm
    SDK_UDP_ExtAxisJog(2, 1, 50, 50, 10)  # 2轴，正转，速度50，加速度50，转动10mm
    SDK_UDP_ExtAxisJog(1, 1, 50, 50, 10)  # 1轴，正转，速度50，加速度50，转动10mm
    SDK_UDP_ExtAxisJog(2, 1, 50, 50, 10)  # 2轴，正转，速度50，加速度50，转动10mm
    SDK_UDP_ExtAxisJog(1, 1, 50, 50, 10)  # 1轴，正转，速度50，加速度50，转动10mm
    SDK_UDP_ExtAxisJog(2, 1, 50, 50, 10)  # 2轴，正转，速度50，加速度50，转动10mm

    SDK_UDP_SetExtAxis(3, 1, error[1], 1)
    print('应用扩展轴3坐标成功！')


def SDK_UDP_ExtAxisSyncMove():
    """
    方法名：ExtAxisSyncMoveJ(joint_pos,desc_pos,tool,user,exaxis_pos, vel=20.0, acc=0.0, ovl= 100.0,  blendT=-1.0, offset_flag=0, offset_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    必须项：
    joint_pos： 目标关节位置，单位 [°]；

    desc_pos：目标笛卡尔位姿，单位 [mm][°]

    tool：工具号，[0~14]

    user：工件号，[0~14]

    exaxis_pos：外部轴 1 位置 ~ 外部轴 4 位

    默认项：
    vel： 速度百分比，[0~100] 默认20.0；

    acc：加速度百分比，[0~100] 暂不开放,默认0.0 ；

    ovl：速度缩放因子，[0~100] 默认100.0 ；

    blendT：[-1.0]-运动到位 (阻塞)，[0~500.0]-平滑时间 (非阻塞)，单位 [ms] 默认-1.0；

    offset_flag：[0]-不偏移，[1]-工件/基坐标系下偏移，[2]-工具坐标系下偏移 默认 0；

    offset_pos：位姿偏移量，单位 [mm][°] 默认[0.0,0.0,0.0,0.0,0.0,0.0] ；
    """
    # 进入自动模式
    robot.Mode(0)
    time.sleep(1)
    robot.SetSpeed(10)

    # 参数设置
    e_pos0 = [0, 0, 0, 0]
    joint_pos0 = [110.897, -94.007, 101.551, -95.441, -90.537, -8.608]
    desc_pos0 = [253.955, -387.336, 210.582, 177.840, -0.216, -150.482]

    e_pos1 = [-30, -30, 0, 0]
    joint_pos1 = [102.893, -69.832, 78.586, -67.281, -85.618, -8.607]
    desc_pos1 = [225.723, -422.843, 173.954, 149.224, 8.196, -161.995]

    # UDP扩展轴与机器人关节运动同步运动
    error = robot.ExtAxisSyncMoveJ(joint_pos0, desc_pos0, 1, 0, e_pos0)
    print("ExtAxisSyncMoveJ", error)
    time.sleep(1)
    error = robot.ExtAxisSyncMoveJ(joint_pos1, desc_pos1, 1, 0, e_pos1)
    print("ExtAxisSyncMoveJ", error)
    time.sleep(3)

    # UDP扩展轴与机器人直线运动同步运动
    error = robot.ExtAxisSyncMoveL(joint_pos1, desc_pos1, 1, 0, e_pos1)
    print("ExtAxisSyncMoveL", error)
    time.sleep(1)
    error = robot.ExtAxisSyncMoveL(joint_pos0, desc_pos0, 1, 0, e_pos0)
    print("ExtAxisSyncMoveL", error)
    time.sleep(3)

    desc_pos_mid = [269.573, -406.937, 210.328, 179.005, -0.272, -149.655]
    desc_pos_end = [289.750, -423.433, 210.361, 178.909, -1.094, -147.689]
    joint_pos_mid = [111.732, -91.378, 99.371, -97.050, -90.418, -8.607]
    joint_pos_end = [113.683, -87.850, 95.829, -97.064, -91.245, -8.608]

    # #UDP扩展轴与机器人圆弧运动同步运动
    time.sleep(3)
    error = robot.ExtAxisSyncMoveC(joint_pos_mid, desc_pos_mid, 1, 0, [0, 10, 0, 0], joint_pos_end, desc_pos_end, 1, 0,
                                   [0, 20, 0, 0])
    print("ExtAxisSyncMoveC", error)
    time.sleep(3)

    error, joint_pos = robot.GetActualJointPosDegree()
    print("GetActualJointPosDegree", error, joint_pos)

    # UDP扩展轴异步运动
    e_pos = [0, 0, 0, 0]

    error = robot.ExtAxisMove(e_pos, 30)
    print("ExtAxisMove返回码为：", error)
    print("joint_pos0的值为：", joint_pos0)
    error = robot.MoveJ(joint_pos0, 1, 0, exaxis_pos=e_pos)
    print("MoveJ返回码为：", error)

    # 进入手动模式
    robot.Mode(1)


def SDK_AuxIO():
    # 设置扩展DO
    error = robot.SetAuxDO(1, True, False, True)
    print("GetAuxAI", error)
    assert error == 0, '扩展IO未正确输入或输出，请检查SDK！！！'

    # 设置扩展AO
    error = robot.SetAuxAO(1, 60, True)
    print("SetAuxAO", error)
    assert error == 0, '扩展IO未正确输入或输出，请检查SDK！！！'

    # 设置扩展DI输入滤波时间
    error = robot.SetAuxDIFilterTime(10)
    print("SetAuxDIFilterTime", error)
    assert error == 0, '扩展IO未正确输入或输出，请检查SDK！！！'

    # 设置扩展AI输入滤波时间
    error = robot.SetAuxAIFilterTime(0, 10)
    print("SetAuxAIFilterTime", error)
    assert error == 0, '扩展IO未正确输入或输出，请检查SDK！！！'

    # 等待扩展DI输入
    error = robot.WaitAuxDI(0, False, 100, False)
    print("WaitAuxDI", error)
    assert error == 0, '扩展IO未正确输入或输出，请检查SDK！！！'

    # 等待扩展AI输入
    error = robot.WaitAuxAI(0, 0, 100, 500, False)
    print("WaitAuxAI", error)
    assert error == 0, '扩展IO未正确输入或输出，请检查SDK！！！'

    # 获取扩展AI值
    error = robot.GetAuxAI(0, False)
    print("GetAuxAI", error)
    assert error[0] == 0, '扩展IO未正确输入或输出，请检查SDK！！！'

    # 获取扩展DI值
    error = robot.GetAuxDI(0, True)
    print("GetAuxDI", error)
    assert error[0] == 0, '扩展IO未正确输入或输出，请检查SDK！！！'


def SDK_485():
    """
    方法名：AuxServoSetParam(servoId,servoCompany,servoModel,servoSoftVersion, servoResolution,axisMechTransRatio)

    servoId：伺服驱动器ID，范围[1-15],对应从站ID；

    servoCompany：伺服驱动器厂商，1-戴纳泰克；

    servoModel：伺服驱动器型号，1-FD100-750C；

    servoSoftVersion：伺服驱动器软件版本，1-V1.0；

    servoResolution：编码器分辨率；

    axisMechTransRatio：机械传动比；

    """
    ret = robot.SetExDevProtocol(4098)  # 设置外设为ModbusMaster
    print("SetExDevProtocol", ret)

    ret, id = robot.GetExDevProtocol()
    print("GetExDevProtocol", ret)
    assert ret == 0 and id == 4098, '设置485连接模式成功'

    ret = robot.AuxServoSetParam(1, 1, 1, 1, 131072, 15.45)  # 设置485扩展轴参数
    print("AuxServoSetParam", ret)
    time.sleep(1)

    ret = robot.AuxServoGetParam(1)  # 获取485扩展轴配置参数
    print("AuxServoGetParam", ret)
    time.sleep(1)

    ret = robot.AuxServoGetStatus(1)  # 查询状态
    print("AuxServoGetStatus", ret)
    time.sleep(1)

    ret = robot.AuxServoClearError(1)  # 清除错误
    print("AuxServoClearError", ret)
    time.sleep(1)

    ret = robot.AuxServoEnable(1, 0)  # 修改控制模式前需去使能 形参第二位：0-去使能， 1-使能
    print("AuxServoEnable(0)", ret)
    time.sleep(3)

    ret = robot.AuxServoSetControlMode(1, 1)  # 设置为速度模式
    print("AuxServoSetControlMode", ret)
    time.sleep(3)

    ret = robot.AuxServoEnable(1, 1)  # 修改控制模式后需使能
    print("AuxServoEnable(1)", ret)
    time.sleep(3)

    ret = robot.AuxServoHoming(1, 1, 10, 10)  # 回零
    print("AuxServoHoming", ret)
    time.sleep(5)

    ret = robot.AuxServoGetStatus(1)  # 查询状态
    print("AuxServoGetStatus", ret)
    time.sleep(1)

    ret = robot.AuxServoSetTargetSpeed(1, 30)  # 速度模式运动，速度30
    print("AuxServoSetTargetSpeed", ret)
    time.sleep(10)

    ret = robot.AuxServoGetStatus(1)  # 查询状态
    print("AuxServoGetStatus", ret)
    time.sleep(1)

    ret = robot.AuxServoSetTargetSpeed(1, 60)  # 速度模式运动，速度60
    print("AuxServoSetTargetSpeed", ret)
    time.sleep(10)
    ret = robot.AuxServoGetStatus(1)  # 查询状态
    print("AuxServoGetStatus", ret)
    time.sleep(1)

    ret = robot.AuxServoSetTargetSpeed(1, -60)  # 速度模式运动，速度60
    print("AuxServoSetTargetSpeed", ret)
    time.sleep(10)
    ret = robot.AuxServoGetStatus(1)  # 查询状态
    print("AuxServoGetStatus", ret)
    time.sleep(1)

    ret = robot.AuxServoSetTargetSpeed(1, 0)  # 结束速度模式运动前应当把速度设为0
    print("AuxServoSetTargetSpeed", ret)
    time.sleep(3)
    ret = robot.AuxServoGetStatus(1)  # 查询状态
    print("AuxServoGetStatus", ret)
    time.sleep(1)


SDK_UDP_LinkExAxis()

SDK_UDP_Calibration()

SDK_UDP_Servo2()

SDK_UDP_CalibrationPoint4()

# SDK_UDP_ExtAxisJog(1, 1, 50, 50, 10)  # 1轴，正转，速度50，加速度50，转动10mm
# SDK_UDP_ExtAxisJog(2, 1, 50, 50, 10)  # 2轴，正转，速度50，加速度50，转动10mm
#
# SDK_UDP_ExtAxisJog(1, 0, 50, 50, 10)  # 1轴，正转，速度50，加速度50，转动10mm
# SDK_UDP_ExtAxisJog(2, 0, 50, 50, 10)  # 2轴，正转，速度50，加速度50，转动10mm

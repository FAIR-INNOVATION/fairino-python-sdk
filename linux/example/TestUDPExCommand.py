from logging import error

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
    """UDP参数配置获取"""
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
    error = robot.SetToolCoord(1, [0, 0, 210, 0, 0, 0], 0, 0)
    print('当前工具变更为：工具1，长度为：210,错误码为：{}'.format(error))

    # 配置UDP通讯参数
    error = robot.ExtDevSetUDPComParam("192.168.58.88", 2021, 2, 50, 5, 50, 1, 100, 10)
    print("ExtDevSetUDPComParam return ", error)
    assert error == 0, '请检查配置参数，或SDK异常！！'

    error,param = robot.ExtDevGetUDPComParam()
    print("ExtDevGetUDPComParam return ", error)
    print("UDP扩展轴通讯参数: ",param)

    # 加载UDP通信
    error = robot.ExtDevLoadUDPDriver()
    print("ExtDevLoadUDPDriver return ", error)
    assert error == 0, '请检查是否正确配置对应参数，或SDK异常！！'


def SDK_UDP_Disconnect():  # 卸载UDP通信
    error = robot.ExtDevUnloadUDPDriver()
    print("ExtDevUnloadUDPDriver return ", error)
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
    print("ExtAxisStartJog return ", error)
    time.sleep(1)
    # UDP扩展轴点动停止
    error = robot.ExtAxisStopJog(axisID)
    print("ExtAxisStopJog return ", error)


def SDK_UDP_Calibration():
    """UDP扩展轴安装位置、DH参数、轴参数配置"""
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
    # error = robot.ExtAxisParamConfig(1, 1, 0, 1000, -1000, 1000, 1000, 1.905, 262144, 200, 0, 0, 0)
    error = robot.ExtAxisParamConfig(axisId=1,axisType=1,axisDirection=0,axisMax=1000,axisMin=-1000,axisVel=1000,axisAcc=1000,axisLead=1.905,encResolution=262144,axisOffect=200,axisCompany=0,axisModel=0,axisEncType=0)
    # error = robot.ExtAxisParamConfig(1, 1, 0, 1000,-1000, 1000, 1000, 1.905, 262144, 200, 0, 0, 0)
    print("UDP扩展轴1参数配置 return:", error)
    error = robot.ExtAxisParamConfig(axisId=2, axisType=1, axisDirection=0, axisMax=1000, axisMin=-1000, axisVel=1000,axisAcc=1000, axisLead=4.444, encResolution=262144, axisOffect=200, axisCompany=0,axisModel=0, axisEncType=0)
    # error = robot.ExtAxisParamConfig(2, 1, 0, 1000, -1000, 1000, 1000, 4.444, 262144, 200, 0, 0, 0)
    print("UDP扩展轴2参数配置 return:", error)


def SDK_UDP_CalibrationPoint4():
    """四点法标定扩展轴坐标系"""
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
    print("GetActualTCPPose return ", error)
    print("获取当前TCP位置信息为：",desc_pos)

    # 设置变位机的标定参考点
    error = robot.SetRefPointInExAxisEnd(desc_pos)
    print("SetRefPointInExAxisEnd return:", error)
    assert error == 0, '请重新标定参考点,或检查SDK逻辑！'

    # 变位机坐标系参考点设置-四点法
    error = robot.PositionorSetRefPoint(1)
    print("PositionorSetRefPoint(1) return:", error)
    time.sleep(2)
    # 进入安全点
    robot.MoveJ(joint_pos=[115.193, -96.149, 92.489, -87.068, -89.15, -83.488], tool=1, user=0)
    SDK_UDP_ExtAxisJog(1, 0, 30, 50, 10)  # 1轴，反转，速度50，加速度50，转动10mm
    SDK_UDP_ExtAxisJog(2, 0, 30, 50, 10)  # 2轴，反转，速度50，加速度50，转动10mm
    robot.MoveJ(joint_pos=[112.239, -90.096, 99.282, -95.909, -89.824, -83.488], tool=1, user=0)  # 点位2
    error = robot.PositionorSetRefPoint(2)
    print("PositionorSetRefPoint(2) return:", error)
    time.sleep(2)
    # 进入安全点
    robot.MoveJ(joint_pos=[115.193, -96.149, 92.489, -87.068, -89.15, -83.488], tool=1, user=0)
    SDK_UDP_ExtAxisJog(1, 0, 30, 50, 10)  # 1轴，反转，速度50，加速度50，转动10mm
    SDK_UDP_ExtAxisJog(2, 0, 30, 50, 10)  # 2轴，反转，速度50，加速度50，转动10mm
    robot.MoveJ(joint_pos=[110.839, -83.473, 93.166, -89.22, -90.499, -83.487], tool=1, user=0)  # 点位3
    error = robot.PositionorSetRefPoint(3)
    print("PositionorSetRefPoint(3) return:", error)
    time.sleep(2)
    # 进入安全点
    robot.MoveJ(joint_pos=[115.193, -96.149, 92.489, -87.068, -89.15, -83.488], tool=1, user=0)
    SDK_UDP_ExtAxisJog(1, 0, 30, 50, 10)  # 1轴，反转，速度50，加速度50，转动10mm
    SDK_UDP_ExtAxisJog(2, 0, 30, 50, 10)  # 2轴，反转，速度50，加速度50，转动10mm
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
    SDK_UDP_ExtAxisJog(1, 1, 30, 50, 10)  # 1轴，正转，速度50，加速度50，转动10mm
    SDK_UDP_ExtAxisJog(2, 1, 30, 50, 10)  # 2轴，正转，速度50，加速度50，转动10mm
    SDK_UDP_ExtAxisJog(1, 1, 30, 50, 10)  # 1轴，正转，速度50，加速度50，转动10mm
    SDK_UDP_ExtAxisJog(2, 1, 30, 50, 10)  # 2轴，正转，速度50，加速度50，转动10mm
    SDK_UDP_ExtAxisJog(1, 1, 30, 50, 10)  # 1轴，正转，速度50，加速度50，转动10mm
    SDK_UDP_ExtAxisJog(2, 1, 30, 50, 10)  # 2轴，正转，速度50，加速度50，转动10mm

    SDK_UDP_SetExtAxis(3, 1, error[1], 1)
    print('应用扩展轴3坐标成功！')


def SDK_UDP_ExtAxisSyncMove():
    """UDP扩展轴异步运动、同步直线和圆弧运动"""
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
    # 加载UPD通讯
    SDK_UDP_LinkExAxis()

    # 伺服初始化
    SDK_UDP_Servo2()

    # 设置标定参数：DH、扩展轴参数、扩展轴位置
    SDK_UDP_Calibration()
    # 进入自动模式
    robot.Mode(0)
    time.sleep(1)
    robot.SetSpeed(10)

    d0 = [311.189, -309.688, 401.836, -174.375, -1.409, -82.354]
    j0 = [118.217, -99.669, 79.928, -73.559, -85.229, -69.359]
    robot.MoveJ(j0, 1, 0)

    # UDP扩展轴与机器人同步运动
    # 参数设置
    e_pos0 = [0, 0, 0, 0]
    robot.ExtAxisMove(e_pos0, 20)

    # joint_pos0 = [115.473,-95.607,105.939,-101.382,-94.664,-71.544]
    # desc_pos0 = [277.996,-367.189,396.365,175.907,-2.469,-82.939]
    #
    # e_pos1 = [-30, -30, 0, 0]
    #
    # joint_pos1 = [106.709,-65.468,69.659,-61.579,-83.506,-71.359]
    # desc_pos1 = [281.385,-539.288,362.729,174.405,32.759,-95.477]
    #
    #
    # # UDP扩展轴与机器人关节运动同步运动
    # error = robot.ExtAxisSyncMoveJ(joint_pos0, desc_pos0, 1, 0, e_pos0)
    # print("ExtAxisSyncMoveJ return ", error)
    # time.sleep(1)
    # error = robot.ExtAxisSyncMoveJ(joint_pos1, desc_pos1, 1, 0, e_pos1)
    # print("ExtAxisSyncMoveJ return ", error)
    # time.sleep(3)




    # UDP扩展轴与机器人直线运动同步运动
    # robot.MoveJ(j0, 1, 0)
    # time.sleep(1)
    # pos4 = [0.000, 0.000, 0.000, 0.000]
    # robot.ExtAxisMove(pos4, 20)
    #
    # j1 = [107.349,-89.537,98.595,-97.909,-88.912,-75.054]
    # d1 = [246.449,-439.816,402.819,-179.244,1.387,-87.604]
    #
    # error = robot.ExtAxisSyncMoveL(j1, d1, 1, 0, pos4)
    # print("ExtAxisSyncMoveL return ", error)
    # time.sleep(1)
    #
    # j2 = [108.615,-64.175,68.829,-68.656,-78.638,-74.998]
    # d2 = [316.116,-551.177,360.849,-176.267 ,27.985,-88.089]
    # pos5 = [-30.000,-30.000, 0.000, 0.000]
    # error = robot.ExtAxisSyncMoveL(j2, d2, 1, 0, pos5)
    # print("ExtAxisSyncMoveL return ", error)
    # time.sleep(3)


    # # #UDP扩展轴与机器人圆弧运动同步运动
    # robot.MoveJ(j0, 1, 0)
    # time.sleep(2)
    # pos5 = [0.000, 0.000, 0.000, 0.000]
    # robot.ExtAxisMove(pos5, 20)
    #
    # jc = [127.992,-105.086,112.329,-97.059,-86.408,-71.264]
    # dc = [321.488,-234.729,402.437,-176.659,1.325,-70.71]
    # robot.MoveJ(jc,1,0)
    # joint_pos_mid = [102.968,-90.629,97.914,-85.055,-81.456,-76.348]
    # desc_pos_mid = [219.404,-425.4,398.991,-174.614,13.894,-90.944]
    # joint_pos_end = [101.952,-62.477,72.267,-77.169,-85.419,-76.345]
    # desc_pos_end = [240.459,-600.829,321.338,178.808,23.025,-92.859]
    # time.sleep(3)
    # error = robot.ExtAxisSyncMoveC(joint_pos_mid, desc_pos_mid, 1, 0, [-15, 0, 0, 0], joint_pos_end, desc_pos_end, 1, 0,
    #                                [-30, 0, 0, 0])
    # print("ExtAxisSyncMoveC return ", error)
    # time.sleep(1)



    # # UDP扩展轴异步运动
    robot.MoveJ(j0, 1, 0)
    time.sleep(2)
    pos6 = [0.000, 0.000, 0.000, 0.000]
    robot.ExtAxisMove(pos6, 20)

    e_pos = [-20.000,-20.000,0,0]
    joint_pos0 = [119.427,-121.379,96.479,-60.089,-85.59,-69.359]
    desc_pos0 = [208.724,-145.2,562.987,-177.644,6.239,-81.278]
    error = robot.MoveJ(joint_pos0, 1, 0)
    robot.ExtAxisMove(e_pos, 20)
    print("MoveJ return ：", error)



    # 机器人和扩展轴回零
    time.sleep(2)
    robot.MoveJ(j0, 1, 0)
    time.sleep(1)
    pos4 = [0.000, 0.000, 0.000, 0.000]
    robot.ExtAxisMove(pos4, 20)
    # 进入手动模式
    robot.Mode(1)


def SDK_AuxIO():
    # 设置扩展DOAO
    # error = robot.SetAuxDO(7, True, False, True)
    # print("GetAuxAI return ", error)
    # assert error == 0, '扩展IO未正确输入或输出，请检查SDK！！！'
    #
    # # 设置扩展AO
    # error = robot.SetAuxAO(1, 30, True)
    # print("SetAuxAO return ", error)
    # assert error == 0, '扩展IO未正确输入或输出，请检查SDK！！！'

    # # 设置扩展DI输入滤波时间
    # error = robot.SetAuxDIFilterTime(10)
    # print("SetAuxDIFilterTime return ", error)
    # assert error == 0, '扩展IO未正确输入或输出，请检查SDK！！！'
    #
    # # 设置扩展AI输入滤波时间
    # error = robot.SetAuxAIFilterTime(0, 10)
    # print("SetAuxAIFilterTime return ", error)
    # assert error == 0, '扩展IO未正确输入或输出，请检查SDK！！！'
    #
    # # 等待扩展DI输入
    # error = robot.WaitAuxDI(0, False, 100, False)
    # print("WaitAuxDI return ", error)
    # assert error == 0, '扩展IO未正确输入或输出，请检查SDK！！！'
    #
    # # 等待扩展AI输入
    # error = robot.WaitAuxAI(0, 0, 100, 500, False)
    # print("WaitAuxAI return ", error)
    # assert error == 0, '扩展IO未正确输入或输出，请检查SDK！！！'
    #
    # # 获取扩展AI值
    # error = robot.GetAuxAI(0, False)
    # print("GetAuxAI return ", error)
    # assert error[0] == 0, '扩展IO未正确输入或输出，请检查SDK！！！'
    #
    # # 获取扩展DI值
    # error = robot.GetAuxDI(0, True)
    # print("GetAuxDI return ", error)
    # assert error[0] == 0, '扩展IO未正确输入或输出，请检查SDK！！！'

    for  i in range(0,128):
        robot.SetAuxDO(i, True, False, False)
        time.sleep(0.05)

    for  i in range(0,128):
        robot.SetAuxDO(i, False, False, False)
        time.sleep(0.05)

    for i in range(1, 4):
        robot.SetAuxAO(i, value=i * 25,block= False)
        time.sleep(0.05)

    time.sleep(2)

    for i in range(1, 4):
        robot.SetAuxAO(i, value=0, block=False )
        time.sleep(0.05)

    while True:
        print("aux DI1 DI0  is ",robot.robot_state_pkg.extDIState[0])



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

def SDK_UDP_Load(self):
    """UDP通讯加载卸载"""
    error = robot.ExtDevUnloadUDPDriver()
    print("ExtDevUnloadUDPDriver return ", error)
    time.sleep(2)
    error = robot.ExtDevLoadUDPDriver()
    print("ExtDevLoadUDPDriver return ", error)
    time.sleep(2)

def SDK_UDP_Pos(self):
    """UDP扩展轴坐标系"""
    tool = 1
    user = 0
    vel = 20
    acc = 100
    ovl = 100
    offset_flag = 0

    exaxisPos = [0, 0, 0, 0]
    offdese = [0, 0, 0, 0, 0, 0]

    d0 = [311.189,-309.688,401.836,-174.375,-1.409,-82.354]
    j0 = [118.217,-99.669,79.928,-73.559,-85.229,-69.359]
    robot.MoveJ(j0, 1, 0)

    # error, desc_pos = robot.GetActualTCPPose()
    # print("GetActualTCPPose return ", error)
    # print("获取当前TCP位置信息为：", desc_pos)
    #
    # # 设置变位机的标定参考点
    # error = robot.SetRefPointInExAxisEnd(desc_pos)
    # print("SetRefPointInExAxisEnd return:", error)
    # assert error == 0, '请重新标定参考点,或检查SDK逻辑！'

    pos1 = [0.000, 0.000, 0.000, 0.000]
    robot.ExtAxisMove(pos1, 20)  # UDP扩展轴运动, 速度100
    descPose1 = [359.526,-516.038,194.469,-175.689,2.781,-87.609]
    jointPos1 = [113.015,-72.49,80.079,-96.505,-84.986,-69.309]
    robot.MoveJ(jointPos1, 1, 0)
    robot.PositionorSetRefPoint(1)

    robot.MoveJ(j0, 1, 0)

    pos2 = [-10.000, -10.000, 0.000, 0.000]
    robot.ExtAxisMove(pos2, 20)
    descPose2 = [333.347,-541.958,164.894,-176.47,4.284,-90.725]
    jointPos2 = [109.989,-69.637,80.146,-97.755,-85.188,-69.269]
    robot.MoveJ(jointPos2,  1, 0)
    robot.PositionorSetRefPoint(2)

    robot.MoveJ(j0, 1, 0)

    pos3 = [-20.000, -20.000, 0.000, 0.00]
    robot.ExtAxisMove(pos3, 20)
    descPose3 = [306.488,-559.238,135.948,-174.925,0.235,-93.517]
    jointPos3 = [107.137,-71.377,87.975,-108.167,-85.169,-69.269]
    robot.MoveJ(jointPos3,  1, 0)
    robot.PositionorSetRefPoint(3)

    robot.MoveJ(j0, 1, 0)

    pos4 = [-30.000, -30.000, 0.000, 0.000]
    robot.ExtAxisMove(pos4, 20)
    descPose4 = [285.528,-569.999,108.568,-174.367,-1.239,-95.643]
    jointPos4 = [105.016,-71.137,92.326,-114.339,-85.169,-69.269]
    robot.MoveJ(jointPos4, 1, 0)
    robot.PositionorSetRefPoint(4)

    error,axisCoord = robot.PositionorComputeECoordSys()
    print("axis coord is ", axisCoord)

    robot.ExtAxisActiveECoordSys(3, 1, axisCoord, 1)

    robot.MoveJ(j0, 1, 0)
    pos4 = [0.000, 0.000, 0.000, 0.000]
    robot.ExtAxisMove(pos4, 20)


def SDK_UDP_Move():
    """UDP扩展轴同步运动"""
    # 加载UPD通讯
    SDK_UDP_LinkExAxis()

    # 伺服初始化
    SDK_UDP_Servo2()

    # 设置标定参数：DH、扩展轴参数、扩展轴位置
    SDK_UDP_Calibration()
    # 进入自动模式
    robot.Mode(0)
    time.sleep(1)
    robot.SetSpeed(10)

    tool = 1
    user = 0
    vel = 20.0
    acc = 100.0
    ovl = 100.0
    blendT = -1
    blendR = -1
    flag = 0
    type = 1

    j1 = [119.296, -84.814, 94.990, -102.876, -87.650, 120.302]
    desc_pos1 = [-34.561, 41.392, 4.130, 179.838, -0.134, -41.525]
    epos1 = [0.001, 0.005, 0.000, 0.000]

    j2 = [108.737, -63.613, 70.933, -88.705, -76.846, 120.302]
    desc_pos2 = [-131.162, 93.384, -10.691, 170.985, -9.220, -40.120]
    epos2 = [-9.999, -9.994, 0.000, 0.000]

    j3 = [105.149, -100.062, 109.340, -98.486, -86.402, 120.309]
    desc_pos3 = [95.535, 114.840, 8.421, -179.261, -2.963, -55.868]
    epos3 = [0.020, 0.005, 0.000, 0.000]

    j4 = [104.394, -82.053, 93.134, -94.266, -87.471, 120.310]
    desc_pos4 = [2.199, 167.136, -11.490, 178.940, -2.507, -44.786]
    epos4 = [-9.979, -9.994, 0.000, 0.000]

    j5 = [105.911, -68.212, 80.904, -95.665, -85.639, 120.311]
    desc_pos5 = [-101.643, 144.925, -22.645, 171.854, 3.405, -32.360]
    epos5 = [-19.978, -19.993, 0.000, 0.000]

    offset_pos = [0, 0, 0, 0, 0, 0]

    rtn = 0
    rtn = robot.ExtAxisSyncMoveL(j1, desc_pos1, tool, user, 100, 100, 100, -1, epos1, 0, offset_pos)
    rtn = robot.ExtAxisSyncMoveL(j2, desc_pos2, tool, user, 100, 100, 100, -1, epos2, 0, offset_pos)

    # UDP扩展轴与机器人关节运动同步运动
    rtn = robot.ExtAxisSyncMoveJ(j3, desc_pos3, tool, user, 100, 100, 100, epos3, blendT, flag, offset_pos)
    rtn = robot.ExtAxisSyncMoveC(j4, desc_pos4, tool, user, 100, 100, epos4, 0, offset_pos, j5, desc_pos5, tool, user,
                                 100, 100, epos5, 0, offset_pos, 100, 0)


def SDK_UDP_WireSearch():
    """UDP通讯PLC上接焊丝寻位"""
    robot.ExtDevSetUDPComParam("192.168.58.2", 2021, 2, 50, 5, 50, 1, 50, 10)
    robot.ExtDevLoadUDPDriver()

    robot.SetWireSearchExtDIONum(0, 0)

    rtn0 = 0
    rtn1 = 0
    rtn2 = 0
    exaxisPos = [0.0, 0.0, 0.0, 0.0]
    offdese = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    descStart = [-158.767, -510.596, 271.709, -179.427, -0.745, -137.349]
    jointStart = [61.667, -79.848, 108.639, -119.682, -89.700, -70.985]

    descEnd = [0.332, -516.427, 270.688, 178.165, 0.017, -119.989]
    jointEnd = [79.021, -81.839, 110.752, -118.298, -91.729, -70.981]

    robot.MoveL(descStart, 1, 0)
    robot.MoveL(descEnd, 1, 0)

    descREF0A = [-66.106, -560.746, 270.381, 176.479, -0.126, -126.745]
    jointREF0A = [73.531, -75.588, 102.941, -116.250, -93.347, -69.689]

    descREF0B = [-66.109, -528.440, 270.407, 176.479, -0.129, -126.744]
    jointREF0B = [72.534, -79.625, 108.046, -117.379, -93.366, -70.687]

    descREF1A = [72.975, -473.242, 270.399, 176.479, -0.129, -126.744]
    jointREF1A = [87.169, -86.509, 115.710, -117.341, -92.993, -56.034]

    descREF1B = [31.355, -473.238, 270.405, 176.480, -0.130, -126.745]
    jointREF1B = [82.117, -87.146, 116.470, -117.737, -93.145, -61.090]

    rtn0 = robot.WireSearchStart(0, 10, 100, 0, 10, 100, 0)
    robot.MoveL(descREF0A, 1, 0, search=0)#起点
    robot.MoveL(descREF0B, 1, 0, search=1)#方向点
    rtn1 = robot.WireSearchWait("REF0")
    rtn2 = robot.WireSearchEnd(0, 10, 100, 0, 10, 100, 0)

    rtn0 = robot.WireSearchStart(0, 10, 100, 0, 10, 100, 0)
    robot.MoveL(descREF1A, 1, 0,search=0)#起点
    robot.MoveL(descREF1B, 1, 0, search=1)#方向点
    rtn1 = robot.WireSearchWait("REF1")
    rtn2 = robot.WireSearchEnd(0, 10, 100, 0, 10, 100, 0)

    rtn0 = robot.WireSearchStart(0, 10, 100, 0, 10, 100, 0)
    robot.MoveL(descREF0A, 1, 0, search=0)#起点
    robot.MoveL(descREF0B, 1, 0, search=1)#方向点
    rtn1 = robot.WireSearchWait("RES0")
    rtn2 = robot.WireSearchEnd(0, 10, 100, 0, 10, 100, 0)

    rtn0 = robot.WireSearchStart(0, 10, 100, 0, 10, 100, 0)
    robot.MoveL(descREF1A, 1, 0, search=0)#起点
    robot.MoveL(descREF1B, 1, 0, search=1)#方向点
    rtn1 = robot.WireSearchWait("RES1")
    rtn2 = robot.WireSearchEnd(0, 10, 100, 0, 10, 100, 0)

    varNameRef = ["REF0", "REF1", "#", "#", "#", "#"]
    varNameRes = ["RES0", "RES1", "#", "#", "#", "#"]
    offectFlag = 0
    offectPos = [0, 0, 0, 0, 0, 0]
    rtn0,offectFlag, offectPos = robot.GetWireSearchOffset(0, 0, varNameRef, varNameRes)
    robot.PointsOffsetEnable(0, offectPos)
    robot.MoveL(descStart, 1, 0)
    robot.MoveL(descEnd, 1, 0)
    robot.PointsOffsetDisable()

# pos4 = [0.000, 0.000, 0.000, 0.000]
# robot.ExtAxisMove(pos4, 20)

# SDK_UDP_LinkExAxis()
# SDK_UDP_Load(robot)#加载卸载
# SDK_UDP_Calibration()#扩展轴DH参数
# SDK_UDP_Servo2()#使能
SDK_UDP_Pos(robot)#四点法定坐标系
# SDK_UDP_ExtAxisSyncMove()#异步运动、同步直线和圆弧运动
# SDK_UDP_WireSearch()#焊丝寻位

















# SDK_UDP_CalibrationPoint4()#四点法定坐标系
# SDK_UDP_Move()
# SDK_AuxIO()#扩展DOI



# SDK_UDP_ExtAxisJog(1, 1, 50, 50, 10)  # 1轴，正转，速度50，加速度50，转动10mm
# SDK_UDP_ExtAxisJog(2, 1, 50, 50, 10)  # 2轴，正转，速度50，加速度50，转动10mm
#
# SDK_UDP_ExtAxisJog(1, 0, 50, 50, 10)  # 1轴，正转，速度50，加速度50，转动10mm
# SDK_UDP_ExtAxisJog(2, 0, 50, 50, 10)  # 2轴，正转，速度50，加速度50，转动10mm

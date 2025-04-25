from fairino import Robot
import time
# A connection is established with the robot controller. A successful connection returns a robot object
robot = Robot.RPC('192.168.58.2')

JP1 = [117.408,-86.777,81.499,-87.788,-92.964,92.959]
DP1 = [327.359,-420.973,518.377,-177.199,3.209,114.449]

JP2 = [72.515,-86.774,81.525,-87.724,-91.964,92.958]
DP2 = [-65.169,-529.17,518.018,-177.189,3.119,69.556]

# JP2_h = [72.515,-86.774,81.525,-87.724,-91.964,92.958]
DP2_h = [-65.169,-529.17,528.018,-177.189,3.119,69.556]

JP3 = [89.281,-102.959,81.527,-69.955,-86.755,92.958]
DP3 = [102.939,-378.069,613.165,176.687,1.217,86.329]

desc = [0,0,0,0,0,0]


def ft_config(self):
    """力传感器参数配置、获取接口"""
    error = robot.FT_SetConfig(company=24,device=0)
    print("FT_SetConfig return ",error)
    time.sleep(1)
    error,config = robot.FT_GetConfig()
    print("传感器编号:", config[0],"力传感器厂商:",config[1],"设备号:",config[2],"软件版本号:",config[3])

def ft_activate(self):
    """进行机器人力传感器激活与零点矫正"""
    error = robot.FT_Activate(state=0)
    print("力传感器复位：", error)
    time.sleep(2)
    error = robot.FT_Activate(state=1)
    print("力传感器激活：", error)
    time.sleep(2)
    error = robot.FT_SetZero(state=0)
    print("去除零点：", error)
    time.sleep(2)
    error = robot.FT_SetZero(state=1)
    print("零点矫正：", error)

def ft_pdidencompute(self):
    """力传感器负载辨识生效"""
    j1 = [-80.009,-97.378,-85.676,-112.937,91.908,-100.777]
    d1 = [-14.404,-455.283,319.847,-172.935,25.141,-68.097]

    j2 = [-81.029,-105.619,-85.677,-86.409,61.741,-100.807]
    d2 = [-107.999,-599.174,285.939,153.472,12.686,-71.284]

    j3 = [-82.669,-108.457,-86.949,-47.156,94.574,-100.806]
    d3 = [6.586,-704.897,309.638,178.909,-27.759,-70.479]
    error = robot.FT_SetRCS(ref=0,coord=[0,0,0,1,0,0])  # 设置参考坐标系为工具坐标系，0-工具坐标系，1-基坐标系
    print('设置参考坐标系错误码', error)
    time.sleep(1)
    tool_id = 10  # 传感器坐标系编号
    tool_coord = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # 传感器相对末端法兰位姿
    tool_type = 1  # 0-工具，1-传感器
    tool_install = 0  # 0-安装末端，1-机器人外部
    error = robot.SetToolCoord(tool_id, tool_coord, tool_type, tool_install)  # 设置传感器坐标系，传感器相对末端法兰位姿
    print('设置传感器坐标系错误码', error)
    time.sleep(1)
    error = robot.FT_PdIdenRecord(tool_id)  # 记录辨识数据
    print('记录负载重量错误码', error)
    time.sleep(1)
    error = robot.FT_PdCogIdenCompute(tool_id)  # 计算负载重量，单位kg
    print('计算负载重量错误码', error)

    # 负载质心辨识，机器人需要示教三个不同的姿态，然后记录辨识数据，最后计算负载质心
    robot.MoveCart(desc_pos=d1,tool=2,user=0)
    error = robot.FT_PdCogIdenRecord(tool_id, 1)
    print('负载质心1错误码', error)  # 记录辨识数据
    robot.MoveCart(desc_pos=d2, tool=2, user=0)
    error = robot.FT_PdCogIdenRecord(tool_id, 2)
    print('负载质心2错误码', error)
    robot.MoveCart(desc_pos=d3, tool=2, user=0)
    error = robot.FT_PdCogIdenRecord(tool_id, 3)
    print('负载质心3错误码', error)
    time.sleep(1)
    error,pos = robot.FT_PdCogIdenCompute()
    print('负载质心计算', pos)

def ft_contol(self):
    """机器人恒力控制"""
    jp1 = [-21.724,-136.814,-59.518,-68.853,89.245,-66.359]
    dp1 = [703.996,-391.695,240.708,-178.756,-4.709,-45.447]

    jp2 = [0.079,-130.285,-71.029,-72.115,88.945,-62.736]
    dp2 = [738.755,-102.812,226.704,177.488,2.566,-27.209]

    robot.FT_Activate(state=1)
    robot.MoveJ(jp1, tool=0, user=0, vel=10)
    error = robot.FT_Control(flag=1,sensor_num=8,select=[0,0,1,0,0,0],force_torque=[0,0,-10,0,0,0],gain=[0.0005,0.0,0.0,0.0,0.0,0.0],adj_sign=0,ILC_sign=0,max_dis=100,max_ang=0)
    print("FT_Control return ", error)
    robot.MoveL(desc_pos=dp2, tool=0, user=0, vel=10)
    error = robot.FT_Control(flag=0,sensor_num=8,select=[0,0,1,0,0,0],force_torque=[0,0,-10,0,0,0],gain=[0.0005,0.0,0.0,0.0,0.0,0.0],adj_sign=0,ILC_sign=0,max_dis=100,max_ang=0)
    print("FT_Control return ",error)
    robot.FT_Activate(state=0)

def ft_gurad(self):
    """力传感器碰撞守护"""
    j1 = [-80.009, -97.378, -85.676, -112.937, 91.908, -100.777]
    d1 = [-14.404, -455.283, 319.847, -172.935, 25.141, -68.097]

    j2 = [-81.029, -105.619, -85.677, -86.409, 61.741, -100.807]
    d2 = [-107.999, -599.174, 285.939, 153.472, 12.686, -71.284]

    j3 = [-82.669, -108.457, -86.949, -47.156, 94.574, -100.806]
    d3 = [6.586, -704.897, 309.638, 178.909, -27.759, -70.479]
    error = robot.FT_Guard(flag=1, sensor_num=8, select=[1, 0, 0, 0, 0, 0], force_torque=[0, 0, 0, 0, 0, 0],
                           max_threshold=[5.0, 0.01, 0.01, 0.01, 0.01, 0.01],
                           min_threshold=[5.0, 0.01, 0.01, 0.01, 0.01, 0.01])
    print("FT_Guard return ", error)
    robot.MoveCart(d1, tool=2, user=0, vel=30)
    robot.MoveCart(d2, tool=2, user=0, vel=30)
    robot.MoveCart(d3, tool=2, user=0, vel=30)
    error = robot.FT_Guard(flag=0, sensor_num=8, select=[1, 0, 0, 0, 0, 0], force_torque=[0, 0, 0, 0, 0, 0],
                           max_threshold=[5.0, 0.01, 0.01, 0.01, 0.01, 0.01],
                           min_threshold=[5.0, 0.01, 0.01, 0.01, 0.01, 0.01])
    print("FT_Guard return ", error)

def ft_compliance(self):
    """力传感器柔顺功能"""
    jp1 = [-21.724, -136.814, -59.518, -68.853, 89.245, -66.359]
    dp1 = [703.996, -391.695, 240.708, -178.756, -4.709, -45.447]

    jp2 = [0.079, -130.285, -71.029, -72.115, 88.945, -62.736]
    dp2 = [738.755, -102.812, 226.704, 177.488, 2.566, -27.209]
    robot.FT_Control(flag=1, sensor_num=8, select=[1, 1, 1, 0, 0, 0], force_torque=[-10, -10, -10, 0, 0, 0],
                     gain=[0.0005, 0.0, 0.0, 0.0, 0.0, 0.0], adj_sign=0, ILC_sign=0, max_dis=100, max_ang=0)
    error = robot.FT_ComplianceStart(p=0.00005,force=10.0)
    print("FT_ComplianceStart return ", error)
    robot.MoveL(dp1, tool=0, user=0, vel=10)
    # time.sleep(2)
    # robot.MoveJ(jp2, tool=0, user=0, vel=10)
    # time.sleep(2)
    # robot.MoveJ(jp1, tool=0, user=0, vel=10)
    # time.sleep(2)
    # robot.MoveJ(jp2, tool=0, user=0, vel=10)
    # time.sleep(2)

    error = robot.FT_ComplianceStop()
    print("FT_ComplianceStop return ", error)
    robot.FT_Control(flag=0, sensor_num=8, select=[1, 1, 1, 0, 0, 0], force_torque=[-10, -10, -10, 0, 0, 0],
                     gain=[0.0005, 0.0, 0.0, 0.0, 0.0, 0.0], adj_sign=0, ILC_sign=0, max_dis=100, max_ang=0)

def ft_drag(self):
    """辅助拖动"""
    error,dragState,sixDimensionalDragState = robot.GetForceAndTorqueDragState()
    print("力传感器辅助拖动控制状态:",dragState,"六维力辅助拖动控制状态:",sixDimensionalDragState)
    robot.EndForceDragControl(status=1,asaptiveFlag=0,interfereDragFlag=0,ingularityConstraintsFlag=0,M=[15.0, 15.0, 15.0, 0.5, 0.5, 0.1],B=[150.0, 150.0, 150.0, 5.0, 5.0, 1.0],K=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],F=[10.0, 10.0, 10.0, 1.0, 1.0, 1.0],Fmax=50,Vmax=100)
    error, dragState, sixDimensionalDragState = robot.GetForceAndTorqueDragState()
    print("力传感器辅助拖动控制状态:", dragState, "六维力辅助拖动控制状态:", sixDimensionalDragState)

    time.sleep(10)
    robot.EndForceDragControl(status=0, asaptiveFlag=0, interfereDragFlag=0,ingularityConstraintsFlag=0, M=[15.0, 15.0, 15.0, 0.5, 0.5, 0.1],
                              B=[150.0, 150.0, 150.0, 5.0, 5.0, 1.0], K=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                              F=[10.0, 10.0, 10.0, 1.0, 1.0, 1.0], Fmax=50, Vmax=100)
    error, dragState, sixDimensionalDragState = robot.GetForceAndTorqueDragState()
    print("力传感器辅助拖动控制状态:", dragState, "六维力辅助拖动控制状态:", sixDimensionalDragState)
    time.sleep(1)

def ft_mixdrag(self):
    """六维力混合拖动"""
    robot.DragTeachSwitch(state=1)
    robot.ForceAndJointImpedanceStartStop(status=1,impedanceFlag=0,lamdeDain=[3.0, 2.0, 2.0, 2.0, 2.0, 3.],KGain=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],BGain=[150.0, 150.0, 150.0, 5.0, 5.0, 1.0],dragMaxTcpVel=1000.0,dragMaxTcpOriVel=180.0)
    error, dragState, sixDimensionalDragState = robot.GetForceAndTorqueDragState()
    print("力传感器辅助拖动控制状态:", dragState, "六维力辅助拖动控制状态:", sixDimensionalDragState)

    time.sleep(10)

    robot.DragTeachSwitch(state=0)
    robot.ForceAndJointImpedanceStartStop(status=0, impedanceFlag=0, lamdeDain=[3.0, 2.0, 2.0, 2.0, 2.0, 3.],
                                          KGain=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                                          BGain=[150.0, 150.0, 150.0, 5.0, 5.0, 1.0], dragMaxTcpVel=1000.0,
                                          dragMaxTcpOriVel=180.0)
    error, dragState, sixDimensionalDragState = robot.GetForceAndTorqueDragState()
    print("力传感器辅助拖动控制状态:", dragState, "六维力辅助拖动控制状态:", sixDimensionalDragState)

def ft_setload(self):
    """力传感器负载设置与获取"""
    error = robot.SetForceSensorPayload(1.34)
    print("SetForceSensorPayload return ", error)
    robot.SetForceSensorPayloadCog(0.778, 2.554, 48.765)
    print("SetForceSensorPayloadCog return ", error)

    error,weight = robot.GetForceSensorPayload()
    print("负载重量:",weight)
    error,x,y,z = robot.GetForceSensorPayloadCog()
    print("负载质心位置：",x,y,z)

def ft_auto(self):
    """力传感器自动校零"""
    robot.SetForceSensorPayload(0.0)
    robot.SetForceSensorPayloadCog(0,0,0)
    error,weight,pos = robot.ForceSensorAutoComputeLoad()
    print("负载重量：",weight,"负载质心位置：",pos)

def ft_test(self):
    company = 17  # 传感器厂商，17-坤维科技
    device = 0  # 传感器设备号
    error = robot.FT_SetConfig(company, device)  # 配置力传感器
    time.sleep(1)
    error = robot.FT_Activate(0)  # 传感器复位
    print("传感器复位错误码", error)
    time.sleep(1)
    error = robot.FT_Activate(1)  # 传感器激活
    print("传感器激活错误码", error)
    time.sleep(1)
    t_coord = [0, 0, 40, 0, 0, 0]
    error = robot.SetToolCoord(8, t_coord, 0, 0)
    tool = 8  # 工具坐标系编号
    user = 0  # 工件坐标系编号

    # 较零点
    joint_pos1 = [-86, -68.043, -125.046, -76.794, 90.897, -70.831]
    ret = robot.MoveJ(joint_pos1, tool, user, vel=100)
    error = robot.FT_SetZero(0)  # 传感器去除零点
    time.sleep(1)
    error = robot.FT_SetZero(1)  # 传感器去除零点
    erro, rcs = robot.FT_GetForceTorqueRCS()  # 查询传感器坐标系下数据
    print(rcs)
    # 恒力控制
    status = 1  # 恒力控制开启标志，0-关，1-开
    sensor_num = 9  # 力传感器编号
    is_select = [0, 0, 1, 0, 0, 0]  # 六个自由度选择[fx,fy,fz,mx,my,mz]，0-不生效，1-生效
    force_torque = [0.0, 0.0, -10.0, 0.0, 0.0, 0.0]
    gain = [0.0008, 0.000001, 0.0, 0.0, 0.0, 0.0]  # 力PID参数，力矩PID参数
    adj_sign = 0  # 自适应启停状态，0-关闭，1-开启
    ILC_sign = 0  # ILC控制启停状态，0-停止，1-训练，2-实操
    max_dis = 100.0  # 最大调整距离
    max_ang = 0.0  # 最大调整角度
    lvbo = 0
    zitai = 0

    joint_p1 = [-98.793, -98.399, -142.686, -29.912, 90.647, -70.953]
    joint_p2 = [-94.065, -124.488, -99.312, -47.574, 90.654, -70.954]
    des_P1 = [-180.808, -328.839, 84.939, -179.713, 1.153, -117.843]
    des_P2 = [-172.343, -613.683, 86.511, -179.830, 1.512, -113.116]

    ret = robot.MoveJ(joint_p1, tool, user, vel=20)
    error = robot.FT_Control(flag= status,sensor_num= sensor_num,select= is_select,force_torque= force_torque,gain= gain,adj_sign= adj_sign,ILC_sign= ILC_sign,max_dis= max_dis,max_ang= max_ang,filter_Sign= lvbo,posAdapt_sign= zitai,isNoBlock=0)
    print("恒力控制开启错误码", error)
    error = robot.MoveL(des_P1, tool, 0)  # 笛卡尔空间直线运动
    print("笛卡尔空间直线运动错误码", error)
    error = robot.MoveL(des_P2, tool, 0)
    print("笛卡尔空间直线运动错误码", error)
    error = robot.MoveL(des_P1, tool, 0)  # 笛卡尔空间直线运动
    print("笛卡尔空间直线运动错误码", error)
    status = 0
    error = robot.FT_Control(flag= status,sensor_num= sensor_num,select= is_select,force_torque= force_torque,gain= gain,adj_sign= adj_sign,ILC_sign= ILC_sign,max_dis= max_dis,max_ang= max_ang,filter_Sign= lvbo,posAdapt_sign= zitai,isNoBlock=0)
    print("恒力控制结束错误码", error)
    # 恒力控制缺少变姿态打磨参数



# ft_config(robot)
# ft_activate(robot)
# ft_pdidencompute(robot)
# ft_contol(robot)
# ft_gurad(robot)
# ft_compliance(robot)
# ft_drag(robot)
# ft_mixdrag(robot)
# ft_setload(robot)
# ft_auto(robot)

ft_test(robot)
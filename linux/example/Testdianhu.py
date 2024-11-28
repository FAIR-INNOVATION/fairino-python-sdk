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

j1 = [-81.684,-106.159,-74.447,-86.33,94.725,41.639]
d1 = [-3.05,-627.474,461.967,179.152,5.565,146.765]
j2 = [-102.804,-106.159,-74.449,-86.328,94.715,41.639]
d2 = [-228.943,-584.228,461.958,179.16,5.559,125.643]

j3 = [-28.529,-140.397,-81.08,-30.934,92.34,-5.629]
d3 = [612.889,-444.619,96.879,-162.269,0.521,-112.455]

j4 = [-11.045,-130.984,-104.495,-12.854,92.475,-5.547]
d4 = [603.806,-217.384,92.227,-158.197,0.239,-94.978]

d5 = [688.259,-566.358,-139.354,-158.206,0.324,-117.817]
d6 = [700.078,-224.752,-149.191,-158.2,0.239,-94.978]

desc = [0,0,0,0,0,0]


def arcstart(self):
    """焊接起弧、收弧"""
    robot.MoveL(desc_pos=d3,tool=0,user=0,vel=30)
    error = robot.ARCStart(ioType=0,arcNum=0,timeout=10000)
    print("ARCStart return ",error)
    robot.MoveL(desc_pos=d4, tool=0, user=0, vel=30)
    error = robot.ARCEnd(ioType=0, arcNum=0, timeout=10000)
    print("ARCEnd return ", error)

def getrelation(self):
    """获取焊接电流、电压与输出模拟量对应关系"""
    robot.WeldingSetCurrentRelation(currentMin=0,currentMax=1000,outputVoltageMin=0,outputVoltageMax=10,AOIndex=0)
    robot.WeldingSetVoltageRelation(weldVoltageMin=0,weldVoltageMax=100,outputVoltageMin=0,outputVoltageMax=10,AOIndex=1)
    error,currentMin,currentMax,outputVoltageMin,outputVoltageMax,AOIndex = robot.WeldingGetCurrentRelation()
    print("焊接电流与输出模拟量对应关系:",currentMin,currentMax,outputVoltageMin,outputVoltageMax,AOIndex)
    error,weldVoltageMin,weldVoltageMax,outputVoltageMin,outputVoltageMax,AOIndex = robot.WeldingGetVoltageRelation()
    print("焊接电压与输出模拟量对应关系:", weldVoltageMin,weldVoltageMax,outputVoltageMin,outputVoltageMax,AOIndex)

def setcv(self):
    """设置焊接电流和电压"""
    error = robot.WeldingSetCurrent(0, 500, 0, 0)
    print("WeldingSetCurrent return ", error)
    error = robot.WeldingSetVoltage(0, 60, 1, 0)
    print("WeldingSetVoltage return ", error)

def setweave(self):
    """设置摆动焊接参数"""
    error = robot.WeaveSetPara(0, 0, 3, 0, 10.0, 0, 0, 0, 100, 100, 50, 1)
    print("WeldingSetProcessParam return ", error)

def weave(self):
    """摆动焊接"""
    robot.MoveL(desc_pos=d5, tool=3, user=0, vel=20)
    robot.WeaveSetPara(0, 0, 1, 0, 10.0, 0, 0, 0, 100, 100, 50, 1)
    robot.ARCStart(ioType=0, arcNum=0, timeout=10000)
    error = robot.WeaveStart(weaveNum=0)
    print("WeaveStart return ", error)
    robot.MoveL(desc_pos=d6, tool=3, user=0, vel=20)
    robot.ARCEnd(ioType=0, arcNum=0, timeout=10000)
    robot.WeaveEnd(weaveNum=0)

def setwir_asp(self):
    """设置送丝送气"""
    time.sleep(2)
    error = robot.SetForwardWireFeed(ioType=0,wireFeed=1)
    print("SetForwardWireFeed return ", error)
    time.sleep(1)
    error = robot.SetForwardWireFeed(ioType=0, wireFeed=0)
    print("SetForwardWireFeed return ", error)
    time.sleep(1)
    error = robot.SetReverseWireFeed(ioType=0, wireFeed=1)
    print("SetReverseWireFeed return ", error)
    time.sleep(1)
    error = robot.SetReverseWireFeed(ioType=0, wireFeed=0)
    print("SetReverseWireFeed return ", error)
    time.sleep(1)

    error = robot.SetAspirated(ioType=0,airControl=1)
    print("SetAspirated return ", error)
    time.sleep(1)
    error = robot.SetAspirated(ioType=0, airControl=0)
    print("SetAspirated return ", error)

def wirepos(self):
    """焊丝寻位"""

    descStart = [153.736,-715.249,-295.037,-179.829,2.613,-155.615]

    descEnd = [73.748,-645.825,-295.016,-179.828,2.608,-155.614]

    robot.MoveL(descStart, 3, 0)
    robot.MoveL(descEnd, 3, 0)

    descREF0A = [273.716,-723.539,-295.075,-179.829,2.608,-155.614]

    descREF0B = [202.588,-723.543,-295.039,-179.829,2.609,-155.614]

    descREF1A = [75.265,-525.091,-295.059,-179.83,2.609,-155.616]

    descREF1B = [75.258,-601.157,-295.075,-179.834,2.609,-155.616]

    error = robot.WireSearchStart(0, 10, 100, 0, 10, 100, 0)
    print("WireSearchStart return ", error)
    robot.MoveL(descREF0A, 3, 0)# 起点
    robot.MoveL(descREF0B, 3, 0,search=1)# 方向点
    error = robot.WireSearchWait("REF0")
    print("WireSearchWait return ", error)
    error = robot.WireSearchEnd(0, 10, 100, 0, 10, 100, 0)
    print("WireSearchEnd return ", error)

    error = robot.WireSearchStart(0, 10, 100, 0, 10, 100, 0)
    robot.MoveL(descREF1A, 3, 0)# 起点
    robot.MoveL(descREF1B, 3, 0,search=1)# 方向点
    error = robot.WireSearchWait("REF1")
    error = robot.WireSearchEnd(0, 10, 100, 0, 10, 100, 0)

    robot.WireSearchStart(0, 10, 100, 0, 10, 100, 0)
    robot.MoveL(descREF0A, 3, 0)# 起点
    robot.MoveL(descREF0B, 3, 0,search=1)# 方向点
    robot.WireSearchWait("RES0")
    robot.WireSearchEnd(0, 10, 100, 0, 10, 100, 0)

    robot.WireSearchStart(0, 10, 100, 0, 10, 100, 0)
    robot.MoveL(descREF1A, 3, 0)# 起点
    robot.MoveL(descREF1B, 3, 0,search=1)# 方向点
    robot.WireSearchWait("RES1")
    robot.WireSearchEnd(0, 10, 100, 0, 10, 100, 0)

    varNameRef = ["REF0", "REF1", "#", "#", "#", "#"]
    varNameRes = ["RES0", "RES1", "#", "#", "#", "#"]

    error,offectFlag,offectpos = robot.GetWireSearchOffset(0, 0, varNameRef, varNameRes)
    robot.PointsOffsetEnable(offectFlag, offectpos)
    robot.MoveL(descStart, 3, 0)
    robot.MoveL( descEnd, 3, 0)
    robot.PointsOffsetDisable()

def wiregyqx(self):
    """焊接工艺曲线设置"""
    error = robot.WeldingSetProcessParam(id=1,startCurrent=177.0,startVoltage=27.0,startTime=1000,weldCurrent=178.0,weldVoltage=28.0,endCurrent=176.0,endVoltage=26.0,endTime=1000)
    print("WeldingSetProcessParam return ", error)
    error,startCurrent,startVoltage,startTime,weldCurrent,weldVoltage,endCurrent,endVoltage,endTime = robot.WeldingGetProcessParam(id=1)
    print("焊接工艺曲线参数:", startCurrent,startVoltage,startTime,weldCurrent,weldVoltage,endCurrent,endVoltage,endTime)

def welddo(self):
    """扩展IO的焊接功能配置"""
    robot.SetArcStartExtDoNum(10)
    robot.SetAirControlExtDoNum(20)
    robot.SetWireForwardFeedExtDoNum(30)
    robot.SetWireReverseFeedExtDoNum(40)

    robot.SetWeldReadyExtDiNum(50)
    robot.SetArcDoneExtDiNum(60)
    robot.SetExtDIWeldBreakOffRecover(70, 80)

def weldchange(self):
    """变姿态的段焊操作"""
    startdescPose = [185.859,-520.154,193.129,-177.129,1.339,-137.789]
    startjointPos = [-60.989,-94.515,-89.479,-83.514,91.957,-13.124]

    enddescPose = [-243.7033,-543.868,143.199,-177.954,1.528,177.758]
    endjointPos = [-105.479,-101.919,-87.979,-78.455,91.955,-13.183]
    robot.SegmentWeldStart(startdescPose, enddescPose, startjointPos, endjointPos, 80, 40, 0, 0, 5000, 1, 0, 3, 0,)

def weldweave(self):
    """摆动预警"""
    DP1 = [-104.846,309.573,336.647,179.681,-0.419,-92.692]
    DP2 = [-318.287,158.502,346.184,179.602,1.081,-46.342]
    robot.MoveL(desc_pos=DP1, tool=3, user=0, vel=20)
    robot.WeaveSetPara(0, 0, 1, 0, 10.0, 0, 0, 0, 100, 100, 50, 1)
    # robot.ARCStart(ioType=0, arcNum=0, timeout=10000)
    # error = robot.WeaveStart(weaveNum=0)
    # print("WeaveStart return ", error)
    robot.WeaveInspectStart(weaveNum=0)
    robot.MoveL(desc_pos=DP2, tool=3, user=0, vel=20)
    robot.WeaveInspectEnd(weaveNum=0)
    # robot.ARCEnd(ioType=0, arcNum=0, timeout=10000)
    # robot.WeaveEnd(weaveNum=0)


# arcstart(robot)
# getrelation(robot)
# setcv(robot)
# setweave(robot)
# weave(robot)
# setwir_asp(robot)
# wirepos(robot)
# wiregyqx(robot)
# welddo(robot)
# weldchange(robot)
# weldweave(robot)
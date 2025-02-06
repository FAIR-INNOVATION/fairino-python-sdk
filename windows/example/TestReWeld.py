from fairino import Robot
import time
# A connection is established with the robot controller. A successful connection returns a robot object
robot = Robot.RPC('192.168.58.2')

def TestReWeld(self):
    rtn = -1
    rtn = robot.WeldingSetCheckArcInterruptionParam(1, 200)
    print("WeldingSetCheckArcInterruptionParam return", rtn)
    rtn = robot.WeldingSetReWeldAfterBreakOffParam(1, 5.7, 98.2, 0)
    print("WeldingSetReWeldAfterBreakOffParam return", rtn)
    enable = 0
    length = 0
    velocity = 0
    moveType = 0
    checkEnable = 0
    arcInterruptTimeLength = 0
    rtn, checkEnable, arcInterruptTimeLength = robot.WeldingGetCheckArcInterruptionParam()
    print("WeldingGetCheckArcInterruptionParam checkEnable:", checkEnable)
    print("WeldingGetCheckArcInterruptionParam arcInterruptTimeLength:", arcInterruptTimeLength)
    rtn, enable, length, velocity, moveType = robot.WeldingGetReWeldAfterBreakOffParam()
    print("*****")
    print("WeldingGetReWeldAfterBreakOffParam enable:", enable)
    print("WeldingGetReWeldAfterBreakOffParam length:", length)
    print("WeldingGetReWeldAfterBreakOffParam velocity:", velocity)
    print("WeldingGetReWeldAfterBreakOffParam moveType:", moveType)

    robot.ProgramLoad("/fruser/test.lua")
    robot.ProgramRun()

    time.sleep(5)

    while True:
        print("welding breakoff state is ", robot.robot_state_pkg.weldingBreakOffState.breakOffState)
        if robot.robot_state_pkg.weldingBreakOffState.breakOffState == 1:
            print("welding breakoff !")
            time.sleep(2)
            rtn = robot.WeldingStartReWeldAfterBreakOff()
            print("WeldingStartReWeldAfterBreakOff return", rtn)
            break
        time.sleep(0.1)

def ExtAxisLaserTracking(self):
    p1Desc = [381.070, -177.767, 227.851, 20.031, -2.455, -111.479]
    p1Joint = [8.383, -44.801, -111.050, -97.707, 78.144, 27.709]

    p2Desc = [381.077, -177.762, 217.865, 20.014, -0.131, -110.631]
    p2Joint = [1.792, -44.574, -113.176, -93.687, 82.384, 21.154]

    p3Desc = [381.070, -177.767, 227.851, 20.031, -2.455, -111.479]
    p3Joint = [8.383, -44.801, -111.050, -97.707, 78.144, 27.709]

    exaxisPos = [0.0, 0.0, 0.0, 0.0]
    offdese = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    exaxisPosStart = [0.0, 0.0, 0.0, 0.0]
    robot.MoveJ(p1Joint, 8, 0, p1Desc)
    robot.ExtAxisMove(exaxisPosStart, 50.0)
    robot.MoveL(p2Desc, 8, 0, p2Joint)
    robot.LaserSensorRecord(4, 1, 10, 2, 35, 0.1, 100)
    exaxisPosTarget = [0.000, 400.015, 0.000, 0.000]
    robot.ExtAxisMove(exaxisPosTarget, 10.0)
    robot.LaserSensorRecord(0, 1, 10, 2, 35, 0.1, 100)
    robot.MoveJ(p3Joint, 8, 0, p3Desc)
    robot.ExtAxisMove(exaxisPosStart, 50.0)

def TestTCP(self):
    p1Desc = [-394.073, -276.405, 399.451, -133.692, 7.657, -139.047]
    p1Joint = [15.234, -88.178, 96.583, -68.314, -52.303, -122.926]

    p2Desc = [-187.141, -444.908, 432.425, 148.662, 15.483, -90.637]
    p2Joint = [61.796, -91.959, 101.693, -102.417, -124.511, -122.767]

    p3Desc = [-368.695, -485.023, 426.640, -162.588, 31.433, -97.036]
    p3Joint = [43.896, -64.590, 60.087, -50.269, -94.663, -122.652]

    p4Desc = [-291.069, -376.976, 467.560, -179.272, -2.326, -107.757]
    p4Joint = [39.559, -94.731, 96.307, -93.141, -88.131, -122.673]

    p5Desc = [-284.140, -488.041, 478.579, 179.785, -1.396, -98.030]
    p5Joint = [49.283, -82.423, 81.993, -90.861, -89.427, -122.678]

    p6Desc = [-296.307, -385.991, 484.492, -178.637, -0.057, -107.059]
    p6Joint = [40.141, -92.742, 91.410, -87.978, -88.824, -122.808]

    exaxisPos = [0, 0, 0, 0]
    offdese = [0, 0, 0, 0, 0, 0]

    posJ = [p1Joint, p2Joint, p3Joint, p4Joint, p5Joint, p6Joint]
    rtn, coordRtn = robot.ComputeToolCoordWithPoints(0, posJ)
    print("ComputeToolCoordWithPoints  ", rtn, "coord is ", coordRtn[0], coordRtn[1],
           coordRtn[2], coordRtn[3], coordRtn[4], coordRtn[5])

    robot.MoveJ(p1Joint, 0, 0, p1Desc)
    robot.SetTcp4RefPoint(1)
    robot.MoveJ(p2Joint, 0, 0, p2Desc)
    robot.SetTcp4RefPoint(2)
    robot.MoveJ(p3Joint, 0, 0, p3Desc)
    robot.SetTcp4RefPoint(3)
    robot.MoveJ(p4Joint, 0, 0, p4Desc)
    robot.SetTcp4RefPoint(4)
    rtn, coordRtn = robot.ComputeTcp4()
    print("ComputeTcp4 ", rtn, "coord is ", coordRtn[0], coordRtn[1],
           coordRtn[2], coordRtn[3], coordRtn[4], coordRtn[5])
    # robot.MoveJ(p5Joint, 0, 0, p5Desc)
    # robot.MoveJ(p6Joint, 0, 0, p6Desc)

def TestTCP6(self):
    p1Desc = [-394.073, -276.405, 399.451, -133.692, 7.657, -139.047]
    p1Joint = [15.234, -88.178, 96.583, -68.314, -52.303, -122.926]

    p2Desc = [-187.141, -444.908, 432.425, 148.662, 15.483, -90.637]
    p2Joint = [61.796, -91.959, 101.693, -102.417, -124.511, -122.767]

    p3Desc = [-368.695, -485.023, 426.640, -162.588, 31.433, -97.036]
    p3Joint = [43.896, -64.590, 60.087, -50.269, -94.663, -122.652]

    p4Desc = [-291.069, -376.976, 467.560, -179.272, -2.326, -107.757]
    p4Joint = [39.559, -94.731, 96.307, -93.141, -88.131, -122.673]

    p5Desc = [-284.140, -488.041, 478.579, 179.785, -1.396, -98.030]
    p5Joint = [49.283, -82.423, 81.993, -90.861, -89.427, -122.678]

    p6Desc = [-296.307, -385.991, 484.492, -178.637, -0.057, -107.059]
    p6Joint = [40.141, -92.742, 91.410, -87.978, -88.824, -122.808]

    exaxisPos = [0, 0, 0, 0]
    offdese = [0, 0, 0, 0, 0, 0]

    posJ = [p1Joint, p2Joint, p3Joint, p4Joint, p5Joint, p6Joint]
    rtn, coordRtn = robot.ComputeToolCoordWithPoints(1, posJ)
    print("ComputeToolCoordWithPoints ", rtn, "coord is ", coordRtn[0], coordRtn[1],
           coordRtn[2], coordRtn[3], coordRtn[4], coordRtn[5])

    robot.MoveJ(p1Joint, 0, 0, p1Desc)
    robot.SetToolPoint(1)
    robot.MoveJ(p2Joint, 0, 0, p2Desc)
    robot.SetToolPoint(2)
    robot.MoveJ(p3Joint, 0, 0, p3Desc)
    robot.SetToolPoint(3)
    robot.MoveJ(p4Joint, 0, 0, p4Desc)
    robot.SetToolPoint(4)
    robot.MoveJ(p5Joint, 0, 0, p5Desc)
    robot.SetToolPoint(5)
    robot.MoveJ(p6Joint, 0, 0, p6Desc)
    robot.SetToolPoint(6)
    rtn, coordRtn = robot.ComputeTool()
    print("ComputeTool ", rtn, "coord is ", coordRtn[0], coordRtn[1],
           coordRtn[2], coordRtn[3], coordRtn[4], coordRtn[5])

def TestWObj(self):
    p1Desc = [-275.046, -293.122, 28.747, 174.533, -1.301, -112.101]
    p1Joint = [35.207, -95.350, 133.703, -132.403, -93.897, -122.768]

    p2Desc = [-280.339, -396.053, 29.762, 174.621, -3.448, -102.901]
    p2Joint = [44.304, -85.020, 123.889, -134.679, -92.658, -122.768]

    p3Desc = [-270.597, -290.603, 83.034, 179.314, 0.808, -114.171]
    p3Joint = [32.975, -99.175, 125.966, -116.484, -91.014, -122.857]

    exaxisPos = [0, 0, 0, 0]
    offdese = [0, 0, 0, 0, 0, 0]

    posTCP = [p1Desc, p2Desc, p3Desc]
    rtn, coordRtn = robot.ComputeWObjCoordWithPoints(1, posTCP, 0)
    print("ComputeWObjCoordWithPoints ", rtn, "coord is ", coordRtn[0], coordRtn[1],
           coordRtn[2], coordRtn[3], coordRtn[4], coordRtn[5])

    robot.MoveJ(p1Joint, 1, 0, p1Desc)
    robot.SetWObjCoordPoint(1)
    robot.MoveJ(p2Joint, 1, 0, p2Desc)
    robot.SetWObjCoordPoint(2)
    robot.MoveJ(p3Joint, 1, 0, p3Desc)
    robot.SetWObjCoordPoint(3)
    rtn, coordRtn = robot.ComputeWObjCoord(1, 0)
    print("ComputeTool ", rtn, "coord is ", coordRtn[0], coordRtn[1],
           coordRtn[2], coordRtn[3], coordRtn[4], coordRtn[5])
    # robot.MoveJ(p5Joint, 0, 0, p5Desc)
    # robot.MoveJ(p6Joint, 0, 0, p6Desc)


def testa(method, pos):
    method = int(method)
    param = {}
    param[0] = pos[0]
    param[1] = pos[1]
    param[2] = pos[2]
    param[3] = pos[3]

    if method == 0:  # 四点法
        param[4] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        param[5] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    else:  # 六点法
        param[4] = pos[4]
        param[5] = pos[5]
    print(param[0])
    print(param[1])
    print(param[2])
    print(param[3])
    print(param[4])
    print(param[5])
    return 0

def test():
    p1Desc = [-394.073, -276.405, 399.451, -133.692, 7.657, -139.047]
    p1Joint = [15.234, -88.178, 96.583, -68.314, -52.303, -122.926]

    p2Desc = [-187.141, -444.908, 432.425, 148.662, 15.483, -90.637]
    p2Joint = [61.796, -91.959, 101.693, -102.417, -124.511, -122.767]

    p3Desc = [-368.695, -485.023, 426.640, -162.588, 31.433, -97.036]
    p3Joint = [43.896, -64.590, 60.087, -50.269, -94.663, -122.652]

    p4Desc = [-291.069, -376.976, 467.560, -179.272, -2.326, -107.757]
    p4Joint = [39.559, -94.731, 96.307, -93.141, -88.131, -122.673]

    p5Desc = [-284.140, -488.041, 478.579, 179.785, -1.396, -98.030]
    p5Joint = [49.283, -82.423, 81.993, -90.861, -89.427, -122.678]

    p6Desc = [-296.307, -385.991, 484.492, -178.637, -0.057, -107.059]
    p6Joint = [40.141, -92.742, 91.410, -87.978, -88.824, -122.808]

    posJ = [p1Joint, p2Joint, p3Joint, p4Joint, p5Joint, p6Joint]
    rtn = testa(0, posJ)
    print(rtn)

# TestReWeld(robot)
# TestTCP(robot)
# TestTCP6(robot)
# TestWObj(robot)

ExtAxisLaserTracking(robot)

# test()

# for i in range (1,31):
    # print("测试焊接中断恢复：",i)
    # TestReWeld(robot)
    # print("四点法工具坐标系测试：", i)
    # TestTCP(robot)
    # print("六点法工具坐标系测试：", i)
    # TestTCP6(robot)
    # print("工件坐标系测试：", i)
    # TestWObj(robot)
    # time.sleep(2)

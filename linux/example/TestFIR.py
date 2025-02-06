from fairino import Robot
import time
# A connection is established with the robot controller. A successful connection returns a robot object
robot = Robot.RPC('192.168.58.2')

def FIRArc(self,enable):
    """ARC运动FIR滤波"""
    startjointPos = [-110.031, -93.517, -111.116, -50.306, 88.224, -81.799]
    startdescPose = [-273.294, -442.769, 341.549, -179.558, -15.157, -118.526]

    midjointPos = [-77.091,-100.464,-100.959,-49.438,80.469,-81.807]
    middescPose = [12.578,-585.573,365.948,173.379,-20.294,-85.709]

    endjointPos = [-51.126, -93.249, -108.367, -50.389, 90.326, -81.804]
    enddescPose = [246.898, -467.876, 367.19, -177.027, -17.755, -59.735]

    exaxisPos = [0, 0, 0, 0]
    offdese = [0, 0, 0, 0, 0, 0]

    if enable:
        robot.LinArcFIRPlanningStart(1000, 1000, 1000, 1000)
        robot.MoveL(startdescPose, 0, 0,vel=100)
        robot.MoveC(desc_pos_p=middescPose,tool_p=0,user_p=0,vel_p=50,desc_pos_t=enddescPose,tool_t=0,user_t=0,vel_t=50)
        robot.LinArcFIRPlanningEnd()
    else:
        robot.MoveL(startdescPose, 0, 0,vel=100)
        robot.MoveC(desc_pos_p=middescPose, tool_p=0, user_p=0,vel_p=50, desc_pos_t=enddescPose, tool_t=0, user_t=0,vel_t=50)

def FIRLin(self,enable):
    """Lin运动FIR滤波"""
    startjointPos = [-110.031, -93.517, -111.116, -50.306, 88.224, -81.799]
    startdescPose = [-273.294, -442.769, 341.549, -179.558, -15.157, -118.526]

    endjointPos = [-51.126, -93.249, -108.367, -50.389, 90.326, -81.804]
    enddescPose = [246.898, -467.876, 367.19, -177.027, -17.755, -59.735]

    exaxisPos = [0, 0, 0, 0]
    offdese = [0, 0, 0, 0, 0, 0]

    if enable:
        robot.LinArcFIRPlanningStart(5000, 5000, 5000, 5000)
        robot.MoveL(startdescPose, 0, 0,vel=100)
        robot.MoveL(enddescPose, 0, 0,vel=100)
        robot.LinArcFIRPlanningEnd()
    else:
        robot.MoveL(startdescPose, 0, 0,vel=100)
        robot.MoveL(enddescPose, 0, 0,vel=100)

def FIRLinL(self,enable):
    """LinL运动FIR滤波"""
    startdescPose = [-608.420, 610.692, 314.930, -176.438, -1.756, 117.333]
    startjointPos = [-56.153, -46.964, 68.015, -113.200, -86.661, -83.479]

    enddescPose = [-366.397, -572.427, 418.339, -178.972, 1.829, -142.970]
    endjointPos = [43.651, -70.284, 91.057, -109.075, -88.768, -83.382]

    exaxisPos = [0, 0, 0, 0]
    offdese = [0, 0, 0, 0, 0, 0]

    if enable:
        robot.LinArcFIRPlanningStart(5000, 5000, 5000, 5000)
        robot.MoveL(startdescPose, 0, 0,vel=100)
        robot.MoveL(enddescPose, 0, 0,vel=100)
        robot.LinArcFIRPlanningEnd()
    else:
        robot.MoveL(startdescPose, 0, 0,vel=100)
        robot.MoveL(enddescPose, 0, 0,vel=100)

def FIRPTP(self,enable):
    """PTP运动FIR滤波"""
    startjointPos = [-110.031,-93.517,-111.116,-50.306,88.224,-81.799]
    startdescPose = [-273.294,-442.769,341.549,-179.558,-15.157,-118.526]

    endjointPos = [-51.126,-93.249,-108.367,-50.389,90.326,-81.804]
    enddescPose = [246.898,-467.876,367.19,-177.027,-17.755,-59.735]

    exaxisPos = [0, 0, 0, 0]
    offdese = [0, 0, 0, 0, 0, 0]

    if enable:
        robot.PtpFIRPlanningStart(maxAcc=400)
        robot.MoveJ(startjointPos, 0, 0,vel=50)
        robot.MoveJ(endjointPos, 0, 0,vel=50)
        robot.PtpFIRPlanningEnd()
    else:
        robot.MoveJ(startjointPos, 0, 0,vel=50)
        robot.MoveJ(endjointPos, 0, 0,vel=50)

robot.LoggerInit()
robot.SetLoggerLevel(lvl=1)

# FIRPTP(robot, False)
# FIRPTP(robot, True)

# FIRLin(robot, False)
# FIRLin(robot, True)

# FIRLinL(robot, False)
# FIRLinL(robot, True)

FIRArc(robot, False)#ARC匀速运动
FIRArc(robot, True)
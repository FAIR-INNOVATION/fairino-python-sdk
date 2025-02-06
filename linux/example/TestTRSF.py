from fairino import Robot
import time
# A connection is established with the robot controller. A successful connection returns a robot object
robot = Robot.RPC('192.168.58.2')

def TRSF(self,enable):
    rtn = 0
    startdescPose = [-226.699, -501.969, 264.638, -174.973, 5.852, 143.301]
    startjointPos = [52.850, -84.327, 102.163, -112.843, -84.131, 0.063]

    enddescPose = [-226.702, -501.973, 155.833, -174.973, 5.852, 143.301]
    endjointPos = [52.850, -77.596, 111.785, -129.196, -84.131, 0.062]

    exaxisPos = [0, 0, 0, 0]
    offdese = [0, 0, 0, 0, 0, 0]

    if enable:
        robot.ToolTrsfStart(1)
        rtn = robot.MoveJ(startjointPos, 0, 0, startdescPose)
        print("rtn is ", rtn)
        rtn = robot.MoveJ(endjointPos, 0, 0, enddescPose)
        print("rtn is ", rtn)
        robot.ToolTrsfEnd()
    else:
        rtn = robot.MoveJ(startjointPos, 0, 0, startdescPose)
        print("rtn is ", rtn)
        rtn = robot.MoveJ(endjointPos, 0, 0, enddescPose)
        print("rtn is ", rtn)

# TRSF(robot,0)
TRSF(robot,1)
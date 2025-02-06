from fairino import Robot
import time
# A connection is established with the robot controller. A successful connection returns a robot object
robot = Robot.RPC('192.168.58.2')



# def test1(self):
#     p1Joint = [74.620, -80.903, 94.608, -109.882, -90.436, -13.432]
#     p1Desc = [72.912, -587.664, 31.849, 43.283, -6.731, 15.068]
#
#     p2Joint = [66.431, -92.875, 116.362, -120.516, -88.627, -24.731]
#     p2Desc = [-104.915, -483.712, -25.231, 42.228, -6.572, 18.433]
#
#     p3Joint = [57.153, -82.046, 104.060, -116.659, -92.478, -24.735]
#     p3Desc = [-242.834, -498.697, -23.681, 46.576, -5.286, 8.318]
#
#     robot.WeldingSetVoltage(1, 19, 0, 0)
#     robot.WeldingSetCurrent(1, 190, 0, 0)
#     robot.MoveJ(joint_pos=p1Joint, tool=1, user=1, vel=100.0, acc=100.0, ovl=100.0)
#     robot.MoveL(desc_pos=p2Desc, tool=1, user=1, vel=100.0, acc=100.0, ovl=50.0, )
#     robot.ARCStart(1, 0, 10000)
#     robot.ArcWeldTraceControl(1, 0, 1, 0.06, 5, 5, 60, 1, 0.06, 5, 5, 60, 0, 0, 4, 1, 10, 2, 2)
#     robot.WeaveStart(0)
#     robot.MoveL(desc_pos=p3Desc, tool=1, user=1, vel=100.0, acc=100.0, ovl=1.0)
#     robot.WeaveEnd(0)
#     robot.ArcWeldTraceControl(0, 0, 1, 0.06, 5, 5, 60, 1, 0.06, 5, 5, 60, 0, 0, 4, 1, 10, 2, 2)
#     robot.ARCEnd(1, 0, 10000)

def test2(self):
    """摆动渐变"""
    p1Joint = [74.620, -80.903, 94.608, -109.882, -90.436, -13.432]
    p1Desc = [-72.912, -587.664, 31.849, 43.283, -6.731, 15.068]

    p2Joint = [66.431, -92.875, 116.362, -120.516, -88.627, -24.731]
    p2Desc = [-104.915, -483.712, -25.231, 42.228, -6.572, 18.433]

    p3Joint = [56.457, -84.796, 104.618, -114.497, -92.422, -25.430]
    p3Desc = [-240.651, -483.840, -7.161, 46.577, -5.286, 8.318]

    robot.WeldingSetVoltage(1, 19, 0, 0)
    robot.WeldingSetCurrent(1, 190, 0, 0)
    robot.MoveJ(joint_pos=p1Joint, tool=1, user=1, vel=100.0, acc=100.0, ovl=100.0)
    robot.MoveL(desc_pos=p2Desc, tool=1, user=1, vel=100.0, acc=100.0, ovl=50.0)
    robot.ARCStart(1, 0, 10000)
    robot.ArcWeldTraceControl(1, 0, 1, 0.06, 5, 5, 60, 1, 0.06, 5, 5, 80, 0, 0, 4, 1, 10, 0, 0)
    robot.WeaveStart(0)
    robot.WeaveChangeStart(1)
    robot.MoveL(desc_pos=p3Desc, tool=1, user=1, vel=100.0, acc=100.0, ovl=1.0)
    robot.WeaveChangeEnd()
    robot.WeaveEnd(0)
    robot.ArcWeldTraceControl(0, 0, 1, 0.06, 5, 5, 60, 1, 0.06, 5, 5, 80, 0, 0, 4, 1, 10, 0, 0)
    robot.ARCEnd(1, 0, 10000)

# test1(robot)
test2(robot)
from fairino import Robot
import time
# A connection is established with the robot controller. A successful connection returns a robot object
robot = Robot.RPC('192.168.58.2')


def CustomCollisionTest(self):
    safety = [5, 5, 5, 5, 5, 5]
    robot.SetCollisionStrategy(3, 1000, 150, 250, safety)
    jAointDetectionThreshould = [0.3, 0.3, 0.3, 0.3, 0.3, 0.3]
    tcpDetectionThreshould = [80, 80, 80, 80, 80, 80]
    rtn = robot.CustomCollisionDetectionStart(3, jAointDetectionThreshould, tcpDetectionThreshould, 0)
    print("CustomCollisionDetectionStart rtn is ", rtn)
    p1Desc = [228.879, -503.594, 453.984, -175.580, 8.293, 171.267]
    p1Joint = [102.700, -85.333, 90.518, -102.365, -83.932, 22.134]

    p2Desc = [-333.302, -435.580, 449.866, -174.997, 2.017, 109.815]
    p2Joint = [41.862, -85.333, 90.526, -100.587, -90.014, 22.135]

    exaxisPos = [0.0, 0.0, 0.0, 0.0]
    offdese = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    while True:
        robot.MoveL(desc_pos=p1Desc, tool=0, user=0, vel=100, acc=100, ovl=100)
        robot.MoveL(desc_pos=p2Desc, tool=0, user=0, vel=100, acc=100, ovl=100)
    rtn = robot.CustomCollisionDetectionEnd()
    print("CustomCollisionDetectionEnd rtn is ", rtn)

CustomCollisionTest(robot)
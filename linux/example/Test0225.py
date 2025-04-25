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

    p1Joint = [93.189,-73.209,104.049,-121.335,-90.439,-13.429]
    p1Desc = [132.497,-558.258,257.293,-179.619,-0.544,-163.382]

    p2Joint = [56.658,-74.379,112.264,-126.895,-90.38,-13.525]
    p2Desc = [-204.705,-495.507,217.007,178.949,-0.139,160.188]

    exaxisPos = [0.0, 0.0, 0.0, 0.0]
    offdese = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    # while True:
    robot.MoveL(desc_pos=p1Desc, tool=0, user=0, vel=100, acc=100, ovl=100)
    robot.MoveL(desc_pos=p2Desc, tool=0, user=0, vel=100, acc=100, ovl=100)
    rtn = robot.CustomCollisionDetectionEnd()
    print("CustomCollisionDetectionEnd rtn is ", rtn)

CustomCollisionTest(robot)
from fairino import Robot
import time
# A connection is established with the robot controller. A successful connection returns a robot object
robot = Robot.RPC('192.168.58.2')

# error = robot.Mode(0)
# print("error is ", error)
def GetSDKVision(self):
    while True:
        # print("aaa")
        error = robot.Mode(0)
        joint_pos = [-25.035,-115.645,-59.515,-89.038,86.888,34.066]
        desc_pos = [563.704,-75.159,447.938,-175.409,-0.633,-120.908]
        # joint_pos = [54.525,-91.408,96.119,-94.696,-88.057,-51.357]

        # error = robot.MoveJ(joint_pos=joint_pos,tool=0,user=0,vel=30)
        # error = robot.MoveL(desc_pos=desc_pos, tool=0, user=0, vel=30)
        print("error is ", error)
        time.sleep(1)
        # print("bbb")
        error = robot.Mode(1)
        joint_pos = [ 0.976,-115.679,-60.537,-89.399,86.875,34.066]
        desc_pos = [543.075,-526.074,392.906,-169.66,3.246,-159.969]
        # joint_pos = [0.007,-91.868,96.111,-88.079,-51.359]
        # error = robot.MoveJ(joint_pos=joint_pos, tool=0, user=0, vel=30)
        # error = robot.MoveL(desc_pos=desc_pos, tool=0, user=0, vel=30)
        print("error is ", error)
        time.sleep(1)
GetSDKVision(robot)
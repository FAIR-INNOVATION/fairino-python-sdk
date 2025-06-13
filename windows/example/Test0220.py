from fairino import Robot
import time
# A connection is established with the robot controller. A successful connection returns a robot object
robot = Robot.RPC('192.168.58.2')


def TestTrajectoryLA(self):
    rtn = 0
    rtn = robot.TrajectoryJUpLoad("D://zUP/A.txt")
    print("TrajectoryJUpLoad A.txt rtn is ",rtn)
    rtn = robot.TrajectoryJUpLoad("D://zUP/B.txt")
    print("TrajectoryJUpLoad B.txt rtn is ", rtn)
    nameA = "/fruser/traj/A.txt"
    nameB = "/fruser/traj/B.txt"

    # rtn = robot.LoadTrajectoryLA(nameA, 2, 0.0, 0, 1.0, 100.0, 200.0, 1000.0) #B样条
    # print("LoadTrajectoryLA rtn is ", rtn)
    robot.LoadTrajectoryLA(nameB, 0, 0, 0, 1, 100, 100, 1000) #直线连接
    # robot.LoadTrajectoryLA(nameA, 1, 2, 0, 2, 100, 200, 1000) #直线拟合
    # error,startPos = robot.GetTrajectoryStartPose(nameA)
    error,startPos = robot.GetTrajectoryStartPose(nameB)
    robot.MoveCart(startPos, 1, 0, 100, 100, 100, -1, -1)
    rtn = robot.MoveTrajectoryLA()
    print("MoveTrajectoryLA rtn is ", rtn)

TestTrajectoryLA(robot)
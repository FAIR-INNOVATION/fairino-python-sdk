from fairino import Robot
# A connection is established with the robot controller. A successful connection returns a robot object
# 与机器人控制器建立连接，连接成功返回一个机器人对象
robot = Robot.RPC('192.168.58.2')
import time


def TestLoadTrajLA(self):
    # Upload trajectory file
    rtn = robot.TrajectoryJUpLoad("D://zUP/horse.txt")
    print(f"Upload TrajectoryJ A {rtn}")

    # Load trajectory with LA parameters
    traj_file_name = "/fruser/traj/horse.txt"
    rtn = robot.LoadTrajectoryLA(traj_file_name, 2, 0, 0, 1, 40, 100, 100, 1)
    print(f"LoadTrajectoryLA {traj_file_name}, rtn is: {rtn}")

    # Get starting pose
    traj_start_pose = [0.0] * 6  # Initialize as list for x,y,z,rx,ry,rz
    rtn, traj_start_pose = robot.GetTrajectoryStartPose(traj_file_name)
    print(f"GetTrajectoryStartPose is: {rtn}")
    print(f"desc_pos:{traj_start_pose[0]},{traj_start_pose[1]},{traj_start_pose[2]},"
          f"{traj_start_pose[3]},{traj_start_pose[4]},{traj_start_pose[5]}")

    time.sleep(1)
    # Move to starting position
    robot.SetSpeed(50)
    robot.MoveCart(traj_start_pose, 0, 0, 100, 100, 100)

    # Execute trajectory
    rtn = robot.MoveTrajectoryLA()
    print(f"MoveTrajectoryLA rtn is: {rtn}")
    robot.CloseRPC()

TestLoadTrajLA(robot)
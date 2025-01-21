from fairino import Robot
import time
# A connection is established with the robot controller. A successful connection returns a robot object
robot = Robot.RPC('192.168.58.2')

def UploadTrajectoryJ(self):
    """上传轨迹J"""
    retval = robot.TrajectoryJDelete("testA.txt")
    print("TrajectoryJDelete return ", retval)
    robot.TrajectoryJUpLoad("D://zUP/testA.txt")

    traj_file_name = "/fruser/traj/testA.txt"
    retval = robot.LoadTrajectoryJ(traj_file_name, 100, 1)
    print("LoadTrajectoryJ return ", retval)

    retval,traj_start_pose = robot.GetTrajectoryStartPose(traj_file_name)
    print("GetTrajectoryStartPose return ", retval)
    print("轨迹起始位姿:", traj_start_pose[0], traj_start_pose[1], traj_start_pose[2], traj_start_pose[3], traj_start_pose[4], traj_start_pose[5])

    robot.SetSpeed(20)
    robot.MoveCart(traj_start_pose, 1, 0)

    time.sleep(5)

    retval,traj_num = robot.GetTrajectoryPointNum()
    print("GetTrajectoryPointNum return ", retval)
    print("轨迹点编号: ", traj_num)

    retval = robot.MoveTrajectoryJ()
    print("MoveTrajectoryJ return ", retval)

def UploadTrajectoryB(self):
    """上传轨迹B"""
    robot.TrajectoryJDelete("testB.txt")
    robot.TrajectoryJUpLoad("D://zUP/testB.txt")

    traj_file_name = "/fruser/traj/testB.txt"
    retval = robot.LoadTrajectoryJ(traj_file_name, 100, 1)
    print("LoadTrajectoryJ return ", retval)

    retval,traj_start_pose = robot.GetTrajectoryStartPose(traj_file_name)
    print("GetTrajectoryStartPose return ", retval)
    print("轨迹起始位姿:", traj_start_pose[0], traj_start_pose[1], traj_start_pose[2], traj_start_pose[3],
          traj_start_pose[4], traj_start_pose[5])

    robot.SetSpeed(20)
    robot.MoveCart(traj_start_pose, 1, 0)
    time.sleep(5)

    retval,traj_num = robot.GetTrajectoryPointNum()
    print("GetTrajectoryPointNum return ", retval)
    print("轨迹点编号: ", traj_num)

    retval = robot.MoveTrajectoryJ()
    print("MoveTrajectoryJ return ", retval)

def MoveRotGripper(self,pos,rotPos):
    """移动旋转夹爪"""
    pos = int(pos)
    rotPos = float(rotPos)
    robot.ResetAllError()
    robot.ActGripper(1, 1)
    time.sleep(1)
    rtn = robot.MoveGripper(index=1,pos=pos,vel=50,force=50,maxtime=5000,block=1,type=1,rotNum=rotPos,rotVel=50,rotTorque=100)
    print("move gripper rtn is ", rtn)
    print("Gripper Motion Done ", pos)

def SetAO(self,value):
    """设置模拟量输出"""
    value = float(value)
    robot.SetAO(0, value, 0)
    robot.SetAO(1, value, 0)
    robot.SetToolAO(0, value, 0)
    while True:
        if abs(((robot.robot_state_pkg.cl_analog_output[0] / 4095) * 100) - value) < 0.5:
            break
        else:
            print("cur AO value is ", robot.robot_state_pkg.cl_analog_output[0])
            time.sleep(0.01)
    print("setAO Done ", value)


robot.LoggerInit()
robot.SetLoggerLevel(lvl=1)

# SetAO(robot,25.0)
# MoveRotGripper(robot, 30, 2.2)

# UploadTrajectoryJ(robot)

while True:
    MoveRotGripper(robot, 30, 0)
    MoveRotGripper(robot, 90, 0)
    UploadTrajectoryJ(robot)
    # MoveRotGripper(robot, 90, 2)
    time.sleep(5)
    MoveRotGripper(robot, 30, 0)
    time.sleep(1)
    MoveRotGripper(robot, 90, 0)
    UploadTrajectoryB(robot)
    # MoveRotGripper(robot, 90, 0)
    time.sleep(5)
    MoveRotGripper(robot, 30, 0)
    print("the robot AO0 ", robot.robot_state_pkg.cl_analog_output[0], ", AO1 ", robot.robot_state_pkg.cl_analog_output[1],
           ", tool AO0 ",robot.robot_state_pkg.tl_analog_output)
    print("gripper pos ", robot.robot_state_pkg.gripper_position," - vel ",
           robot.robot_state_pkg.gripper_speed," - torque ", robot.robot_state_pkg.gripper_current,
           " - rotPos ", robot.robot_state_pkg.gripperRotNum," - rotvel - ", robot.robot_state_pkg.gripperRotSpeed," rotTor - ", robot.robot_state_pkg.gripperRotTorque)



# while True:
    # print("the robot AO0 ", robot.robot_state_pkg.cl_analog_output[0], ", AO1 ", robot.robot_state_pkg.cl_analog_output[1],
    #            ", tool AO0 ",robot.robot_state_pkg.tl_analog_output)
    # print("gripper pos ", robot.robot_state_pkg.gripper_position," - vel ",
    #        robot.robot_state_pkg.gripper_speed," - torque ", robot.robot_state_pkg.gripper_current,
    #        " - rotPos ", robot.robot_state_pkg.gripperRotNum," - rotvel - ", robot.robot_state_pkg.gripperRotSpeed," rotTor - ", robot.robot_state_pkg.gripperRotTorque)


# robot.CloseRPC()
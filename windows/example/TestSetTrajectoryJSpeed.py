from time import sleep
import time
from fairino import Robot

# 与机器人控制器建立连接
robot = Robot.RPC('192.168.58.2')


def TestSetTrajectoryJSpeed(self):
    # 上传轨迹文件
    rtn = robot.TrajectoryJUpLoad("C://Users/lenovo/Desktop/trajHelix_aima_1.txt")
    print(f"Upload TrajectoryJ A {rtn}")

    traj_file_name = "/fruser/traj/trajHelix_aima_1.txt"
    # 加载轨迹文件，参数：文件名，速度百分比，是否循环（1:循环）
    rtn = robot.LoadTrajectoryJ(name=traj_file_name, ovl=100, opt=1)
    print(f"LoadTrajectoryJ {traj_file_name}, rtn is: {rtn}")

    # 获取轨迹起始点位姿
    rtn, traj_start_pose = robot.GetTrajectoryStartPose(name=traj_file_name)
    print(f"GetTrajectoryStartPose is: {rtn}")
    print(
        f"desc_pos:{traj_start_pose[0]},{traj_start_pose[1]},{traj_start_pose[2]},{traj_start_pose[3]},{traj_start_pose[4]},{traj_start_pose[5]}")

    time.sleep(1)

    # 设置基础速度并移动到轨迹起始点
    robot.SetSpeed(50)
    robot.MoveCart(desc_pos=traj_start_pose, tool=0, user=0, vel=100, acc=100, ovl=100, blendT=-1, config=-1)

    # 获取轨迹点数
    rtn, traj_num = robot.GetTrajectoryPointNum()
    print(f"GetTrajectoryStartPose rtn is: {rtn}, traj num is: {traj_num}")

    # 开始执行轨迹运动
    rtn = robot.MoveTrajectoryJ()
    print(f"MoveTrajectoryJ rtn is: {rtn}")

    time.sleep(1)

    # 获取机器人实时状态
    trajspeedMode = 0
    while True:
        rtn, pkg = robot.GetRobotRealTimeState()
        if pkg.motion_done != 0:
            break

        # 设置轨迹速度为10%
        rtn = robot.SetTrajectoryJSpeed(ovl=10.0, mode=trajspeedMode)
        print(f"SetTrajectoryJSpeed is: {rtn}")

        time.sleep(1)

        # 设置轨迹速度为80%
        rtn = robot.SetTrajectoryJSpeed(ovl=80.0, mode=trajspeedMode)
        print(f"SetTrajectoryJSpeed is: {rtn}")

        time.sleep(1)

    # 关闭连接
    robot.CloseRPC()
    time.sleep(1)


# 调用测试函数
TestSetTrajectoryJSpeed(robot)
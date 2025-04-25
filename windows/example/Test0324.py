from fairino import Robot
import time
# A connection is established with the robot controller. A successful connection returns a robot object
import time
import threading

# 与机器人控制器建立连接，连接成功返回一个机器人对象
robot = Robot.RPC('192.168.58.2')

# 打印错误码的辅助函数
def check_error(ret, action_name):
    if ret != 0:  # 假设0表示无错误，具体的错误码请根据机器人控制器的文档进行确认
        print(f"{action_name} 失败，错误码：{ret}")
    else:
        print(f"{action_name} 成功")

# 机器人运动任务
def perform_robot_motion():
    # 定义目标位置
    desc_pos1 = [-240.458,10.459,363.469,-178.61,0.282,156.73]
    desc_posc1 = [-317.289,-327.946,369.197,-171.279,2.725,-144.059]  # MoveC过渡点
    desc_posc2 = [-436.862,90.475,380.839,178.928,7.759,163.584]  # MoveC目标点

    # 工具坐标系编号和工件坐标系编号
    tool = 1
    user = 2

    # 进行笛卡尔空间直线运动
    ret = robot.MoveL(desc_pos1, tool, user, vel=30, acc=100, blendR=1.0)
    check_error(ret, "笛卡尔空间直线运动")

    # 进行笛卡尔空间圆弧运动
    ret = robot.Circle(desc_posc1, tool, user, desc_posc2, tool, user, vel_t=10, offset_flag=1,
                       offset_pos=[5, 10, 15, 0, 0, 1])
    check_error(ret, "笛卡尔空间圆弧运动")


# 使用线程控制停止运动（如果必要的话）
def stop_motion_after_delay():
    time.sleep(6)  # 等待6秒
    ret = robot.StopMove()  # 停止机器人运动
    check_error(ret, "终止运动")

# 启动机器人运动任务线程
motion_thread = threading.Thread(target=perform_robot_motion)

# 启动停止运动的线程
stop_thread = threading.Thread(target=stop_motion_after_delay)

# 启动线程
motion_thread.start()
stop_thread.start()

# 等待线程完成
motion_thread.join()
stop_thread.join()

print("所有任务完成")
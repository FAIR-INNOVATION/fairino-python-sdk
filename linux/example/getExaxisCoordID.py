from fairino import Robot
from fairino.Robot import RobotState, SetRobotRealtimeStateConfig, DEFAULT_CNDE_STATES, AddRobotRealtimeState, DeleteRobotRealtimeState, SetRobotRealtimeStatePeriod
import time


def main():
    # 添加需要获取的实时状态数据：扩展轴坐标系ID

    rtn = AddRobotRealtimeState([RobotState.ExaxisCoordID])
    if rtn != 0:
        print(f"✗ 添加字段失败，错误码: {rtn}")
        return None
    print("✓ 字段添加成功")

    robot = Robot.RPC('192.168.58.2')
    time.sleep(0.5)  # 等待连接和数据接收

    while True:
        rtn, pkg = robot.GetRobotRealTimeState()
        # 获取扩展轴坐标系ID和坐标值
        exaxis_coord_id = pkg.exaxisCoordID
        exaxis_coord = pkg.exAxisCoord
        print(
            f"id is {exaxis_coord_id}, coord is {exaxis_coord[0]} {exaxis_coord[1]} {exaxis_coord[2]} {exaxis_coord[3]} {exaxis_coord[4]} {exaxis_coord[5]}")
        time.sleep(0.1)  # 100ms

    # 关闭连接（正常情况下不会执行到这里，因为while循环是无限的）
    robot.CloseRPC()


# 调用测试函数
main()
from fairino import Robot
from fairino.Robot import RobotState
import time


def main():
    # 添加需要获取的实时状态数据（如果需要的话）
    # rtn = AddRobotRealtimeState([RobotState.ExaxisCoordID])
    # if rtn != 0:
    #     print(f"✗ 添加字段失败，错误码: {rtn}")
    #     return None
    # print("✓ 字段添加成功")

    # 与机器人控制器建立连接
    robot = Robot.RPC('192.168.58.2')
    time.sleep(0.5)  # 等待连接和数据接收

    # 配置UDP通讯参数
    rtn = robot.ExtDevSetUDPComParam("192.168.58.88", 2021, 2, 100, 3, 200, 1, 100, 5, 1)
    print(f"ExtDevSetUDPComParam rtn is {rtn}")

    # 获取UDP通讯参数
    error, param = robot.ExtDevGetUDPComParam()
    print("ExtDevGetUDPComParam return ", error)
    print("UDP扩展轴通讯参数: ", param)

    # 加载UDP驱动
    robot.ExtDevLoadUDPDriver()

    # 设置扩展轴命令完成时间
    rtn = robot.SetExAxisCmdDoneTime(5000.0)
    print(f"SetExAxisCmdDoneTime rtn is {rtn}")

    # 扩展轴伺服使能
    rtn = robot.ExtAxisServoOn(1, 1)
    print(f"ExtAxisServoOn axis id 1 rtn is {rtn}")
    rtn = robot.ExtAxisServoOn(2, 1)
    print(f"ExtAxisServoOn axis id 2 rtn is {rtn}")
    time.sleep(2)

    # 扩展轴回零
    robot.ExtAxisSetHoming(1, 0, 10, 2)
    time.sleep(2)
    rtn = robot.ExtAxisSetHoming(2, 0, 10, 2)
    print(f"ExtAxisSetHoming rtn is {rtn}")

    time.sleep(4)

    # 设置机器人相对扩展轴位置
    rtn = robot.SetRobotPosToAxis(1)
    print(f"SetRobotPosToAxis rtn is {rtn}")

    # 设置扩展轴DH参数配置
    rtn = robot.SetAxisDHParaConfig(10, 20, 0, 0, 0, 0, 0, 0, 0)
    print(f"SetAxisDHParaConfig rtn is {rtn}")

    # 配置扩展轴1参数
    rtn = robot.ExtAxisParamConfig(1, 1, 1, 1000, -1000, 1000, 1000, 1.905, 262144, 200, 1, 0, 0)
    print(f"ExtAxisParamConfig axis 1 rtn is {rtn}")

    # 获取扩展轴1参数配置
    rtn, axisType, axisDirection, axisMax, axisMin, axisVel, axisAcc, axisLead, encResolution, axisOffect, axisCompany, axisModel, axisEncType = robot.ExtAxisGetParamConfig(1)
    print(f"axis id 1 ExtAxisGetParamConfig : axisType {axisType}, axisDirection {axisDirection}, axisMax {axisMax}, axisMin {axisMin}, axisVel {axisVel}, axisAcc {axisAcc}, axisLead {axisLead}, encResolution {encResolution}, axisOffect {axisOffect}, axisCompany {axisCompany}, axisModel {axisModel}, axisEncType {axisEncType}")

    # 配置扩展轴2参数
    rtn = robot.ExtAxisParamConfig(2, 1, 1, 1000, -1000, 1000, 1000, 4.444, 262144, 200, 1, 0, 0)
    print(f"ExtAxisParamConfig axis 2 rtn is {rtn}")

    # 获取扩展轴2参数配置
    rtn, axisType, axisDirection, axisMax, axisMin, axisVel, axisAcc, axisLead, encResolution, axisOffect, axisCompany, axisModel, axisEncType = robot.ExtAxisGetParamConfig(2)
    print(f"axis id 2 ExtAxisGetParamConfig : axisType {axisType}, axisDirection {axisDirection}, axisMax {axisMax}, axisMin {axisMin}, axisVel {axisVel}, axisAcc {axisAcc}, axisLead {axisLead}, encResolution {encResolution}, axisOffect {axisOffect}, axisCompany {axisCompany}, axisModel {axisModel}, axisEncType {axisEncType}")

    time.sleep(3)

    # 扩展轴1点动测试
    robot.ExtAxisStartJog(1, 0, 10, 10, 30)
    time.sleep(1)
    robot.ExtAxisStopJog(1)
    time.sleep(3)
    robot.ExtAxisServoOn(1, 0)

    time.sleep(3)

    # 扩展轴2点动测试
    robot.ExtAxisStartJog(2, 0, 10, 10, 30)
    time.sleep(1)
    robot.ExtAxisStopJog(2)
    time.sleep(3)
    robot.ExtAxisServoOn(2, 0)

    # 卸载UDP驱动
    robot.ExtDevUnloadUDPDriver()

    # 关闭连接
    robot.CloseRPC()


# 调用测试函数
main()
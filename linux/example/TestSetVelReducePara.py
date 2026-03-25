from time import sleep
import time
from fairino import Robot

# 与机器人控制器建立连接
robot = Robot.RPC('192.168.58.2')

def TestSetVelReducePara(self):
    # 初始化关节位置、外部轴和偏移
    j1 = [0, -90, 90, 0, 0, 0]
    j2 = [90, -90, 90, 0, 0, 0]
    epos = [0, 0, 0, 0]
    offset_pos = [0, 0, 0, 0, 0, 0]

    # 设置基础速度
    robot.SetSpeed(80)

    # 测试参数错误的情况（mode=2 无效？）
    rtn = robot.SetVelReducePara(2, 30, 1)
    print(f"SetVelReducePara param error rtn is {rtn}")

    # 关闭减速功能（mode=0, action=1 表示禁用减速）
    rtn = robot.SetVelReducePara(0, 30, 1)
    print(f"SetVelReducePara disable reduce vel rtn is {rtn}")
    robot.MoveJ(joint_pos=j1, tool=0, user=0, vel=100, acc=100, ovl=100,
                exaxis_pos=epos, blendT=-1, offset_flag=0, offset_pos=offset_pos)
    robot.MoveJ(joint_pos=j2, tool=0, user=0, vel=100, acc=100, ovl=100,
                exaxis_pos=epos, blendT=-1, offset_flag=0, offset_pos=offset_pos)

    # 启用减速功能（mode=1, action=1）
    rtn = robot.SetVelReducePara(1, 30, 1)
    print(f"SetVelReducePara reduce vel rtn is {rtn}")
    robot.MoveJ(joint_pos=j1, tool=0, user=0, vel=100, acc=100, ovl=100,
                exaxis_pos=epos, blendT=-1, offset_flag=0, offset_pos=offset_pos)
    robot.MoveJ(joint_pos=j2, tool=0, user=0, vel=100, acc=100, ovl=100,
                exaxis_pos=epos, blendT=-1, offset_flag=0, offset_pos=offset_pos)

    # 测试 action=2（可能表示急停或禁用机器人）
    rtn = robot.SetVelReducePara(2, 30, 2)
    print(f"SetVelReducePara disable robot rtn is {rtn}")
    robot.MoveJ(joint_pos=j1, tool=0, user=0, vel=100, acc=100, ovl=100,
                exaxis_pos=epos, blendT=-1, offset_flag=0, offset_pos=offset_pos)
    robot.MoveJ(joint_pos=j2, tool=0, user=0, vel=100, acc=100, ovl=100,
                exaxis_pos=epos, blendT=-1, offset_flag=0, offset_pos=offset_pos)

    # 等待、复位错误并重新使能机器人
    time.sleep(2)
    robot.ResetAllError()
    robot.RobotEnable(1)
    time.sleep(1)

    # 测试 action=0（可能表示仅上报错误，不执行动作）
    rtn = robot.SetVelReducePara(2, 30, 0)
    print(f"SetVelReducePara report error rtn is {rtn}")
    robot.MoveJ(joint_pos=j1, tool=0, user=0, vel=100, acc=100, ovl=100,
                exaxis_pos=epos, blendT=-1, offset_flag=0, offset_pos=offset_pos)
    robot.MoveJ(joint_pos=j2, tool=0, user=0, vel=100, acc=100, ovl=100,
                exaxis_pos=epos, blendT=-1, offset_flag=0, offset_pos=offset_pos)

    # 关闭连接
    robot.CloseRPC()

TestSetVelReducePara(robot)
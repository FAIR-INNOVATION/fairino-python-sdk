from time import sleep

from fairino import Robot
from ctypes import sizeof
# A connection is established with the robot controller. A successful connection returns a robot object
# 与机器人控制器建立连接,连接成功返回一个机器人对象
robot = Robot.RPC('192.168.58.2')

import time
def TestRotInsert(self):
    forceInsertion = 2.0  # 力或力矩阈值（0~100），单位N或Nm
    angleMax = 12  # 最大旋转角度，单位°
    orn = 2  # 力的方向，1-fz,2-mz
    angAccmax = 5.0  # 最大旋转角加速度，单位°/s^2,暂不使用
    status = 1  # 恒力控制开启标志，0-关，1-开
    sensor_num = 2  # 力传感器编号
    gain = [0.0001, 0.0, 0.0, 0.0, 0.0, 0.0]  # 最大阈值
    adj_sign = 0  # 自适应启停状态，0-关闭，1-开启
    ILC_sign = 0  # ILC控制启停状态，0-停止，1-训练，2-实操
    max_dis = 1000.0  # 最大调整距离
    max_ang = 5.0  # 最大调整角度
    ft = [0.0] * 6  # [fx, fy, fz, tx, ty, tz]
    rcs = 0  # 参考坐标系，0-工具坐标系，1-基坐标系
    angVelRot = 2.0  # 旋转角速度，单位°/s
    rotorn = 1  # 旋转方向，1-顺时针，2-逆时针

    j1 = [58.417, -85.578, -100.516, -83.915, 90.000, -31.662]
    j2 = [58.417, -87.111, -107.956, -74.942, 90.000, -31.662]
    desc_p1 = [328.796, 339.109, 433.617, 179.993, 0.005, 0.079]
    desc_p2 = [328.795, 339.109, 373.605, 179.993, 0.005, 0.079]
    epos = [0.0] * 4
    offset_pos = [0.0] * 6

    robot.MoveL(joint_pos=j1, desc_pos=desc_p1, tool=0, user=0, vel=100.0, acc=180.0, ovl=100.0, blendR=-1.0, exaxis_pos=epos, search=0, offset_flag=1, offset_pos=offset_pos)
    select3 = [0, 0, 1, 0, 0, 0]  # 六个自由度选择[fx,fy,fz,mx,my,mz]，0-不生效，1-生效
    ft[2] = -10.0  # fz = -10.0
    gain[0] = 0.0001
    status = 1
    robot.FT_Control(flag=status, sensor_id=sensor_num, select=select3, ft=ft, ft_pid=gain, adj_sign=adj_sign, ILC_sign=ILC_sign, max_dis=max_dis, max_ang=max_ang)
    rtn = robot.FT_RotInsertion(rcs=rcs, angVelRot=angVelRot, ft=forceInsertion, max_angle=angleMax, orn=orn, max_angAcc=angAccmax, rotorn=rotorn, strategy=0)
    print(f"FT_RotInsertion rtn is {rtn}")
    status = 0
    robot.FT_Control(flag=status, sensor_id=sensor_num, select=select3, ft=ft, ft_pid=gain, adj_sign=adj_sign, ILC_sign=ILC_sign, max_dis=max_dis, max_ang=max_ang)
    robot.MoveL(joint_pos=j2, desc_pos=desc_p2, tool=0, user=0, vel=100.0, acc=180.0, ovl=100.0, blendR=-1.0, exaxis_pos=epos, search=0, offset_flag=1, offset_pos=offset_pos)
    time.sleep(1)
    error,pkg = robot.GetRobotRealTimeState()
    print(f"robot errcode {pkg.main_code},{pkg.sub_code}")
    robot.CloseRPC()
    return 0

def test(self):
    error = robot.SetLoadWeight(0,0)
    print(error)

test(robot)

# TestRotInsert(robot)
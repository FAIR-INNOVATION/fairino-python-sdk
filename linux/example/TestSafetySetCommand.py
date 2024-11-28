from fairino import Robot
import time
# A connection is established with the robot controller. A successful connection returns a robot object
robot = Robot.RPC('192.168.58.2')

JP1 = [117.408,-86.777,81.499,-87.788,-92.964,92.959]
DP1 = [327.359,-420.973,518.377,-177.199,3.209,114.449]

JP2 = [72.515,-86.774,81.525,-87.724,-91.964,92.958]
DP2 = [-65.169,-529.17,518.018,-177.189,3.119,69.556]

# JP2_h = [72.515,-86.774,81.525,-87.724,-91.964,92.958]
DP2_h = [-65.169,-529.17,528.018,-177.189,3.119,69.556]

JP3 = [89.281,-102.959,81.527,-69.955,-86.755,92.958]
DP3 = [102.939,-378.069,613.165,176.687,1.217,86.329]

desc = [0,0,0,0,0,0]

def setanticollision(self):
    """机器人设置碰撞等级测试"""
    error = robot.SetAnticollision(mode=0,level=[2.0,2.0,2.0,2.0,2.0,2.0],config=1)
    print("SetAnticollision return ", error)
    error = robot.SetCollisionStrategy(strategy=0,safeTime=1000,safeDistance=150,safetyMargin=[10,10,10,10,10,10])
    print("SetCollisionStrategy return ", error)
    # while True:
    #     robot.MoveJ(JP1, tool=0, user=0, vel=30)
    #     robot.MoveJ(JP2, tool=0, user=0, vel=30)

def setlimitpositive(self):
    """机器人设置关节限位测试"""
    error = robot.SetLimitPositive(p_limit=[170.0,80.0,150.0,80.0,170.0,160.0])
    print("SetLimitPositive return ", error)
    error = robot.SetLimitNegative(n_limit=[-170.0,-260.0,-150.0,-260.0,-170.0,-160.0])
    print("SetLimitNegative return ", error)

def resetallerror(self):
    """机器人清除错误测试"""
    robot.MoveJ(JP1, tool=20, user=0, desc_pos=DP1, vel=30)
    time.sleep(3)
    error = robot.ResetAllError()
    print("ResetAllError return ", error)

def setfrictionvalue_level(self):
    """机器人设置摩擦力补偿系数测试-正装"""
    error = robot.SetFrictionValue_level(coeff=[0.5,0.5,0.5,0.5,0.5,0.5])
    print("SetFrictionValue_level return ", error)
    error = robot.FrictionCompensationOnOff(state=1)
    print("FrictionCompensationOnOff return ", error)

def setfrictionvalue_wall(self):
    """机器人设置摩擦力补偿系数测试-侧装"""
    error = robot.FrictionCompensationOnOff(state=1)
    print("FrictionCompensationOnOff return ", error)
    error = robot.SetFrictionValue_wall(coeff=[0.5,0.5,0.5,0.5,0.5,0.5])
    print("SetFrictionValue_wall return ", error)

def setfrictionvalue_ceiling(self):
    """机器人设置摩擦力补偿系数测试-倒装"""
    error = robot.FrictionCompensationOnOff(state=1)
    print("FrictionCompensationOnOff return ", error)
    error = robot.SetFrictionValue_ceiling(coeff=[0.5,0.5,0.5,0.5,0.5,0.5])
    print("SetFrictionValue_ceiling return ", error)

def setfrictionvalue_freedom(self):
    """机器人设置摩擦力补偿系数测试-自由安装"""
    error = robot.FrictionCompensationOnOff(state=1)
    print("FrictionCompensationOnOff return ", error)
    error = robot.SetFrictionValue_freedom(coeff=[0.5,0.5,0.5,0.5,0.5,0.5])
    print("SetFrictionValue_freedom return ", error)

def setstaticcollisionstrategy(self):
    """机器人设置碰撞后继续运行测试"""
    error = robot.SetStaticCollisionOnOff(status=1)
    print("SetCollisionStrategy return ", error)

def setcollisionstrategy(self):
    """机器人关节扭矩功率检测测试"""
    robot.DragTeachSwitch(state=1)
    error = robot.SetPowerLimit(status=1,power=2)
    print("SetPowerLimit return ", error)
    error,joinytor = robot.GetJointTorques()
    print("GetJointTorques return ", error)
    print("当前关节扭矩为：", joinytor)

    count = 10
    robot.ServoJTStart()
    while count > 0:
        # joinytor[0] += 0.1
        error = robot.ServoJT(torque=joinytor,interval=0.001)
        count -= 1
        time.sleep(0.01)
    robot.ServoJTEnd()



# setanticollision(robot)
# setlimitpositive(robot)
# resetallerror(robot)
# setfrictionvalue_level(robot)
# setfrictionvalue_wall(robot)
# setfrictionvalue_ceiling(robot)
# setfrictionvalue_freedom(robot)
# setstaticcollisionstrategy(robot)
# setcollisionstrategy(robot)
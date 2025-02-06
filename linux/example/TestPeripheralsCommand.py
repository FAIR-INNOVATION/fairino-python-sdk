# from example.servo import joint_pos
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


j1 = [-81.684,-106.159,-74.447,-86.33,94.725,41.639]
d1 = [-3.05,-627.474,461.967,179.152,5.565,146.765]
j2 = [-102.804,-106.159,-74.449,-86.328,94.715,41.639]
d2 = [-228.943,-584.228,461.958,179.16,5.559,125.643]

desc = [0,0,0,0,0,0]


def setgripperconfig(self):
    """夹爪参数配置"""
    error = robot.SetGripperConfig(company=3,device=0)
    print("SetGripperConfig return ",error)
    time.sleep(1)
    error,gripperconfig = robot.GetGripperConfig()
    print("夹爪参数：", gripperconfig)
    print("GetGripperConfig return ",error)

def actgripper(self):
    """进行夹爪激活、控制夹爪运动和获取夹爪运动完成状态"""
    error = robot.ActGripper(index=1,action=0)
    print("ActGripper return ", error)
    time.sleep(1)
    error = robot.ActGripper(index=1, action=1)
    print("ActGripper return ", error)
    time.sleep(2)

    error = robot.MoveGripper(index=1,pos=20,vel=50,force=50,maxtime=30000,
                              block=0,type=0,rotNum=0,rotVel=0,rotTorque=0)
    print("MoveGripper return ", error)
    time.sleep(3)

    error,state = robot.GetGripperMotionDone()
    print("GripperMotionDone ", state)

def computepick(self):
    """进行抓取点和撤退点的计算"""
    error,prepick = robot.ComputePrePick(desc_pos=d2,zlength=10,zangle=0)
    print("抓取点位姿：", prepick)
    print("ComputePrePick return ", error)

    error, prepick = robot.ComputePostPick(desc_pos=d2, zlength=10, zangle=0)
    print("撤退点位姿：", prepick)
    print("ComputePostPick return ", error)

def grippartest(self):
    """设置末端传感器配置参数、获取参数、激活和写入寄存器参数"""
    error = robot.AxleSensorConfig(idCompany=18,idDevice=0,idSoftware=0,idBus=1)
    print("AxleSensorConfig return ", error)

    error,company,device = robot.AxleSensorConfigGet()
    print("厂商：", company)
    print("类型：", device)
    print("AxleSensorConfigGet return ", error)

    error = robot.AxleSensorActivate(actFlag=1)
    print("AxleSensorActivate return ", error)

    error = robot.AxleSensorRegWrite(devAddr=1,regHAddr=4,regLAddr=6,regNum=1,data1=0,data2=0,isNoBlock=0)
    print("AxleSensorRegWrite return ", error)



# setgripperconfig(robot)
# actgripper(robot)
# computepick(robot)
grippartest(robot)
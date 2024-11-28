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

def getrobotinstallangle(self):
    """获取机器人安装角度测试"""
    robot.SetRobotInstallAngle(yangle=90,zangle=90)
    error,installangle = robot.GetRobotInstallAngle()
    print("GetRobotInstallAngle return ", error)
    print("当前安装角度为：", installangle)

def getinversekin(self):
    """逆向运动学计算测试"""
    error,jp = robot.GetInverseKin(type=0,desc_pos=DP1)
    print("GetInverseKin return ", error)
    print("逆向运动学计算结果为：", jp)
    print("原始逆向运动学为：", JP1)

def getinversekinhassolution(self):
    """逆向运动学计算是否有解测试"""
    error,i = robot.GetInverseKinHasSolution(type=0,desc_pos=DP1,joint_pos_ref=JP1)
    print("GetInverseKinHasSolution return ", error)
    print("逆向运动学计算是否有解为：", i)

def getforwardkin(self):
    """正向运动学计算测试"""
    error,dp = robot.GetForwardKin(joint_pos=JP2)
    print("GetForwardKin return ", error)
    print("正向运动学计算结果为：", dp)
    print("原始正向运动学为：", DP2)

def getversion(self):
    """获取硬固件版本信息测试"""
    error,hardversion1,hardversion2,hardversion3,hardversion4,hardversion5,hardversion6,hardversion7,hardversion8 = robot.GetSlaveHardVersion()
    print("GetSlaveHardVersion return ", error)
    print("硬件信息为：", hardversion1,hardversion2,hardversion3,hardversion4,hardversion5,hardversion6,hardversion7,hardversion8)
    error, firmversion1, firmversion2, firmversion3, firmversion4, firmversion5, firmversion6, firmversion7, firmversion8 = robot.GetSlaveFirmVersion()
    print("GetSlaveHardVersion return ", error)
    print("固件信息为：", firmversion1, firmversion2, firmversion3, firmversion4, firmversion5, firmversion6, firmversion7, firmversion8)

def getsshkeygen(self):
    """获取SSH公钥测试"""
    error,sshkeygen = robot.GetSSHKeygen()
    print("GetSSHKeygen return ", error)
    print("SSH公钥为：", sshkeygen)

def gettargetpayload(self):
    """获取当前负载重量测试"""
    error,targetpayload = robot.GetTargetPayload()
    print("GetTargetPayload return ", error)
    print("当前负载重量为：", targetpayload)
    error,targetpos = robot.GetTargetPayloadCog()
    print("当前负载质心坐标为：", targetpos)

def getrobotcurjointsconfig(self):
    """获取机器人当前关节配置测试"""
    error,robotcurjointsconfig = robot.GetRobotCurJointsConfig()
    print("GetRobotCurJointsConfig return ", error)
    print("机器人当前关节配置为：", robotcurjointsconfig)

def getsystemclock(self):
    """获取系统时间测试"""
    error,systemclock = robot.GetSystemClock()
    print("GetSystemClock return ", error)
    print("系统时间为：", systemclock)

def getdefaulttransvel(self):
    """获取默认速度测试"""
    error,defaulttransvel = robot.GetDefaultTransVel()
    print("GetDefaultTransVel return ", error)
    print("默认速度为：", defaulttransvel)

def gettcpoffset(self):
    """获取当前工具坐标系测试"""
    error,tcpoffset = robot.GetTCPOffset()
    print("GetTCPOffset return ", error)
    print("当前工具坐标系为：", tcpoffset)

def getwobjoffset(self):
    """获取当前工件坐标系测试"""
    error,wobjoffset = robot.GetWObjOffset()
    print("GetWObjOffset return ", error)
    print("当前工件坐标系为：", wobjoffset)

def getjointsoftlimitdeg(self):
    """获取关节软限位角度测试"""
    error,jointsoftlimitdeg = robot.GetJointSoftLimitDeg()
    print("GetJointSoftLimitDeg return ", error)
    print("关节软限位角度为：", jointsoftlimitdeg)

def getrobotmotiondone(self):
    """查询机器人运动是否完成测试"""
    error,robotmotiondone = robot.GetRobotMotionDone()
    print("GetRobotMotionDone return ", error)
    print("机器人运动是否完成为：", robotmotiondone)

def getrobotteachingpoint(self):
    """查询机器人示教管理点位数据测试"""
    error,robotteachingpoint = robot.GetRobotTeachingPoint(name="P1")
    print("GetRobotTeachingPoint return ", error)
    print("机器人示教管理点位数据为：", robotteachingpoint)




# getrobotinstallangle(robot)
# getinversekin(robot)
# getinversekinhassolution(robot)
# getforwardkin(robot)
# getversion(robot)
# getsshkeygen(robot)
gettargetpayload(robot)
# getrobotcurjointsconfig(robot)
# getsystemclock(robot)
# getdefaulttransvel(robot)
# gettcpoffset(robot)
# getwobjoffset(robot)
# getjointsoftlimitdeg(robot)
# getrobotmotiondone(robot)
# getrobotteachingpoint(robot)
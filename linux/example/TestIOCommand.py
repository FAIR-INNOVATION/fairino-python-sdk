from fairino import Robot
import time
# A connection is established with the robot controller. A successful connection returns a robot object
robot = Robot.RPC('192.168.58.2')

j1 = [-81.684,-106.159,-74.447,-86.33,94.725,41.639]
d1 = [-3.05,-627.474,461.967,179.152,5.565,146.765]
j2 = [-102.804,-106.159,-74.449,-86.328,94.715,41.639]
d2 = [-228.943,-584.228,461.958,179.16,5.559,125.643]

desc = [0,0,0,0,0,0]

def setdo(self):
    """机器人SetDO测试"""
    error = robot.SetDO(id=0,status=1)
    time.sleep(2)
    error = robot.SetDO(id=0, status=0)
    print("SetDO return ",error)

def settooldo(self):
    """机器人SetToolDO测试"""
    time.sleep(1)
    error = robot.SetToolDO(id=0,status=1)
    time.sleep(3)
    error = robot.SetToolDO(id=0, status=0)
    print("SetToolDO return ",error)

def setao(self):
    """机器人SetAO测试"""
    error = robot.SetAO(id=0,value=0.6)
    time.sleep(3)
    error = robot.SetAO(id=0, value=0.2)
    print("SetAO return ",error)

def settoolao(self):
    """机器人SetToolAO测试"""
    error = robot.SetToolAO(id=0,value=0.6)
    time.sleep(3)
    error = robot.SetToolAO(id=0, value=0.2)
    print("SetToolAO return ",error)

def waitdi(self):
    """机器人WaitDI测试"""
    print("WaitDI start")
    error = robot.WaitDI(id=1,status=1,maxtime=5000,opt=0)
    print("WaitDI return ",error)

def waitmultidi(self):
    """机器人WaitMultiDI测试"""
    print("WaitMultiDI start")
    error = robot.WaitMultiDI(mode=0,id = 6,status=6,maxtime=5000,opt=0)
    print("WaitMultiDI return ",error)

def waittooldi(self):
    """机器人WaitToolDI测试"""
    print("WaitToolDI start")
    error = robot.WaitToolDI(id = 0,status=1,maxtime=10000,opt=0)
    print("WaitToolDI return ",error)

def waitai(self):
    """机器人WaitAI测试"""
    print("WaitAI start")
    error = robot.WaitAI(id=0,sign=0,value=8,maxtime=5000,opt=2)
    print("WaitAI return ",error)

def waittoolai(self):
    """机器人WaitToolAI测试"""
    print("WaitToolAI start")
    error = robot.WaitToolAI(id = 0,sign=0,value=8,maxtime=5000,opt=2)
    print("WaitToolAI return ",error)

def moveaostart(self):
    """机器人MoveAOStart测试"""
    error = robot.MoveAOStart(0,100,98,1)
    print("MoveAOStart", error)
    time.sleep(3)
    error, joint_pos = robot.GetActualJointPosDegree()
    print("GetActualJointPosDegree", joint_pos)
    joint_pos[0] = joint_pos[0] - 10
    # 机器人关节运动
    error = robot.MoveJ(j1, 0, 0)
    print("MoveJ", error)
    time.sleep(3)
    # 控制箱运动AO停止
    error = robot.MoveAOStop()
    print("MoveAOStop", error)

def movetoolaostart(self):
    """机器人MoveToolAOStart测试"""
    error = robot.MoveToolAOStart(0, 100, 98, 1)
    print("MoveToolAOStart", error)
    time.sleep(3)
    error, joint_pos = robot.GetActualJointPosDegree()
    print("GetActualJointPosDegree", joint_pos)
    joint_pos[0] = joint_pos[0] + 10
    # 机器人关节运动
    error = robot.MoveJ(joint_pos, 0, 0)
    print("MoveJ", error)
    time.sleep(3)
    # 控制箱运动AO停止
    error = robot.MoveToolAOStop()
    print("MoveToolAOStop", error)

def resetboxdo(self):
    """控制箱、末端AO复位测试"""
    robot.SetAO(id=0,value=30)
    robot.SetToolAO(id=0,value=30)
    time.sleep(3)

    robot.SetOutputResetCtlBoxAO(resetFlag=1)
    robot.SetOutputResetAxleAO(resetFlag=1)

    robot.MoveJ(j1, 0, 0, vel=30)
    robot.MoveToolAOStop()
    robot.MoveAOStop()


def reaodo(self):
    """扩展AODO复位测试"""
    # robot.ExtDevUnloadUDPDriver()
    # time.sleep(2)
    # robot.ExtDevLoadUDPDriver()
    # time.sleep(2)

    robot.SetAuxDO(DONum=0,bOpen=True,smooth=False,block=False)
    robot.SetAuxDO(DONum=1, bOpen=True, smooth=False, block=False)
    robot.SetAuxDO(DONum=2, bOpen=True, smooth=False, block=False)
    robot.SetAuxDO(DONum=3, bOpen=True, smooth=False, block=False)

    robot.SetAuxAO(AONum=0,value=1234,block=False)
    robot.SetAuxAO(AONum=1,value=2345, block=False)
    robot.SetAuxAO(AONum=2,value=3456, block=False)
    robot.SetAuxAO(AONum=3,value=1111, block=False)

    time.sleep(2)

    robot.SetOutputResetExtDO(resetFlag=1)
    robot.SetOutputResetExtAO(resetFlag=1)


# setdo(robot)
# settooldo(robot)
# setao(robot)
# settoolao(robot)

################还没测#####################
# waitdi(robot)
# waitmultidi(robot)
# waittooldi(robot)
# waitai(robot)
# waittoolai(robot)
# moveaostart(robot)
# movetoolaostart(robot)
# resetboxdo(robot)
reaodo(robot)
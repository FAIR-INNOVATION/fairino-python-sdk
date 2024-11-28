from fairino import Robot
import time
# A connection is established with the robot controller. A successful connection returns a robot object
robot = Robot.RPC('192.168.58.2')

def rpc(self):
    """RPC通信测试"""
    if robot.is_conect:
        print("连接成功")
    else:
        print("连接失败")

def getsdkversion(self):
    """SDK版本号获取测试"""
    error,sdk = robot.GetSDKVersion()
    print("SDK版本:%s，%s" % (sdk[0],sdk[1]))

def getcontrollerip(self):
    """控制器IP获取测试"""
    error,IP = robot.GetControllerIP()
    print("控制器IP:%s" % IP)

def dragteachswitch(self):
    """机器人拖动模式设置测试"""
    print("进入拖动示教模式")
    robot.DragTeachSwitch(state=1)
    time.sleep(3)
    print("退出拖动示教模式")
    robot.DragTeachSwitch(state=0)

def isindragteach(self):
    """查询机器人是否处于拖动示教模式测试"""
    error,IsDrag = robot.IsInDragTeach()
    if IsDrag==0:
        print("机器人处于非拖动示教模式")
    elif IsDrag==1:
        print("机器人处于拖动示教模式")

def robotenable(self):
    """机器人上使能或下使能测试"""
    print("机器人下使能")
    robot.RobotEnable(state=0)
    time.sleep(3)
    print("机器人上使能")
    robot.RobotEnable(state=1)
    time.sleep(3)
    print("机器人下使能")
    robot.RobotEnable(state=0)


def mode(self):
    """机器人模式切换测试"""
    print("机器人自动模式")
    robot.Mode(state=0)
    time.sleep(3)
    print("机器人手动模式")
    robot.Mode(state=1)


# rpc(robot)
# getsdkversion(robot)



getcontrollerip(robot)



# dragteachswitch(robot)
# isindragteach(robot)
# robotenable(robot)
# mode(robot)

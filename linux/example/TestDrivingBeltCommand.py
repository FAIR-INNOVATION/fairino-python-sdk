from fairino import Robot
# from fairino.pyd import Robot
import time
# A connection is established with the robot controller. A successful connection returns a robot object
robot = Robot.RPC('192.168.58.2')

# JP1 = [117.408,-86.777,81.499,-87.788,-92.964,92.959]
# DP1 = [327.359,-420.973,518.377,-177.199,3.209,114.449]
#
# JP2 = [72.515,-86.774,81.525,-87.724,-91.964,92.958]
# DP2 = [-65.169,-529.17,518.018,-177.189,3.119,69.556]
#
# # JP2_h = [72.515,-86.774,81.525,-87.724,-91.964,92.958]
# DP2_h = [-65.169,-529.17,528.018,-177.189,3.119,69.556]
#
# JP3 = [89.281,-102.959,81.527,-69.955,-86.755,92.958]
# DP3 = [102.939,-378.069,613.165,176.687,1.217,86.329]
#
# desc = [0,0,0,0,0,0]


def auxiliarysensor(self):
    """辅助传感器设备"""
    error = robot.AxleSensorConfig(idCompany=18,idDevice=0,idSoftware=0,idBus=1)
    print("AxleSensorConfig return ", error)
    error,idCompany,idDevice = robot.AxleSensorConfigGet()
    print("AxleSensorConfigGet return ", error)
    print("厂商: ",idCompany,"类型: ",idDevice)
    errror = robot.AxleSensorActivate(1)
    print("AxleSensorActivate return ", error)
    error = robot.AxleSensorRegWrite(1, 4, 6, 1, 0, 0, 0)
    print("AxleSensorRegWrite return ", error)

def grippertest(self):
    """夹爪测试"""
    company = 4
    device = 0
    softversion = 0
    bus = 1
    index = 2
    act = 0
    max_time = 30000
    block = 0
    status = -1
    fault = -1
    rtn = -1
    deviceID = -1
    gripperConfig = [company, device, softversion, bus]
    robot.SetGripperConfig(company=company,device=device,softversion=softversion,bus=bus)
    time.sleep(1)
    error,getConfig = robot.GetGripperConfig()
    print("gripper config :", getConfig[0], " , ", getConfig[1], " , ", getConfig[2], " , ", getConfig[3])
    error = robot.ActGripper(index=index,action=act)
    print("ActGripper return : ", error)
    time.sleep(1)
    act = 1
    error = robot.ActGripper(index=index,action=act)
    print("ActGripper return : ", error)
    time.sleep(1)
    error = robot.MoveGripper(index=index,pos=20,speed=50,force=50,maxtime=max_time,block=block)
    print("MoveGripper return : ", error)
    time.sleep(2)
    error = robot.MoveGripper(index=index, pos=40, speed=50, force=0, maxtime=max_time, block=block)
    print("MoveGripper return : ", error)
    time.sleep(4)
    print("gripper motion done : ",robot.robot_state_pkg.gripper_motiondone,", ",robot.robot_state_pkg.gripper_fault)

    desc_pos1 = [-350.959, 87.828, 353.444, -179.689, -0.142, 2.474]
    error,desc_pos2 = robot.ComputePrePick(desc_pos1, 10, 0)
    print("ComputePrePick: ",desc_pos2)
    error,desc_pos3 = robot.ComputePostPick(desc_pos1, 10, 0)
    print("ComputePrePick: ",desc_pos3)

def drivingreferencepoint(self):
    """传送带参考点"""
    # error = robot.ConveyorPointIORecord()#记录IO监测点
    # print("ConveyorPointIORecord return ", error)
    #
    # error = robot.ConveyorPointARecord()#记录A点
    # print("ConveyorPointARecord return ", error)
    #
    # error = robot.ConveyorRefPointRecord()#记录参考点
    # print("ConveyorRefPointRecord return ", error)
    #
    # error = robot.ConveyorPointBRecord()#记录B点
    # print("ConveyorPointBRecord return ", error)

    # error = robot.ConveyorSetParam(param=[1, 10000, 2.0, 0, 1, 20])#传动带参数配置
    # print("ConveyorSetParam return ", error)

def drivingbelt(self):
    """传送带跟踪抓取"""
    # pos1 = [-351.206,87.966,351.048,-179.933,-0.509,2.471]
    # pos2 = [-351.203,-213.393,351.054,-179.932,-0.508,2.472]

    # pos0 = [-353.803,-3.677,241.444,177.937,1.184,-1.197]
    # error = robot.MoveCart(pos0, 3, 0)

    pos1 = [-351.549,87.914,354.176,-179.679,-0.134,2.468]
    pos2 = [-351.558,-247.286,354.131,-179.679,-0.142,2.474]
    # error = robot.MoveCart(pos2, 3, 0)

    error = robot.ConveyorCatchPointComp(cmp = [0.0, 0.0, 0.0])#抓取点补偿
    if error != 0:
        return
    print("ConveyorCatchPointComp return ",error)

    error = robot.MoveCart(pos1, 1, 0)
    print("MoveCart return ", error)
    error = robot.ConveyorIODetect(max_t=10000)#传送带工件IO检测
    print("ConveyorIODetect return ", error)
    error = robot.ConveyorGetTrackData(mode=1)#获取物体当前位置
    print("ConveyorGetTrackData return ", error)
    error = robot.ConveyorTrackStart(status=1)#传动带跟踪开始
    print("ConveyorTrackStart return ", error)
    error = robot.ConveyorTrackMoveL("cvrCatchPoint", 1, 0)#传送带跟踪直线运动
    # error = robot.ConveyorTrackMoveL("cvrCatchPoint", 3, 0)
    print("ConveyorTrackMoveL return ", error)
    error = robot.MoveGripper(1, 60, 60, 30, 30000, 0)#控制夹爪
    print("MoveGripper return ", error)
    error = robot.ConveyorTrackMoveL("cvrRaisePoint", 1, 0)#传送带跟踪直线运动
    # error = robot.ConveyorTrackMoveL("cvrRaisePoint", 3, 0)
    print("ConveyorTrackMoveL return ", error)
    error = robot.ConveyorTrackEnd()#传动带跟踪停止
    print("ConveyorTrackEnd return ", error)
    error = robot.MoveCart(pos2, 1, 0)
    print("MoveCart return ", error)
    error = robot.MoveGripper(1, 100, 60, 30, 30000, 0)#控制夹爪
    print("MoveGripper return ", error)

def closeRPC(self, robot):
    """CloseRPC功能测试"""
    # robot = Robot.RPC('192.168.58.2')
    robot.CloseRPC()

    time.sleep(1)
    robot = Robot.RPC('192.168.58.2')
    error = robot.Mode(state=0)
    print("Mode return ", error)
    time.sleep(2)
    error = robot.Mode(state=1)
    print("Mode return ", error)


# drivingreferencepoint(robot)
# drivingbelt(robot)
closeRPC(robot,robot=robot)

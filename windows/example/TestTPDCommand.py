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

def settpd(self):
    """TPD轨迹示教测试"""
    robot.Mode(state=1)
    robot.DragTeachSwitch(state=1)
    robot.SetTPDParam(name="test1009",period_ms=2)
    error,tcp = robot.GetActualTCPPose()
    error = robot.SetTPDStart(name="test1009",period_ms=2)
    print("SetTPDStart return ", error)
    time.sleep(10)
    robot.SetWebTPDStop()
    robot.DragTeachSwitch(state=0)
    robot.SetTPDDelete(name="test1009")
    print("SetTPDDelete return ", error)

def movetpd(self):
    """TPD轨迹示教测试"""
    robot.Mode(state=1)
    robot.DragTeachSwitch(state=1)
    robot.SetTPDParam(name="test1009", period_ms=2)
    robot.SetTPDStart(name="test1009", period_ms=2 )
    time.sleep(10)
    robot.SetWebTPDStop()
    robot.DragTeachSwitch(state=0)

    robot.LoadTPD(name="test1009")
    error,pos = robot.GetTPDStartPose(name="test1009")
    robot.MoveCart(pos,tool=0,user=0, vel=30)
    error = robot.MoveTPD(name="test1009",blend=1,ovl=100)
    robot.SetTPDDelete(name="test1009")
    print("MoveTPD return ", error)

def loadtrajectoryj(self):
    """轨迹加载复现测试"""
    error = robot.SetTrajectoryJForceTorque(ft=[10,10,10,10,10,10])
    print("SetTrajectoryJForceTorque return ", error)
    error = robot.SetTrajectoryJForceFx(fx=10)
    print("SetTrajectoryJForceFx return ", error)
    error = robot.SetTrajectoryJForceFy(fy=10)
    print("SetTrajectoryJForceFy return ", error)
    error = robot.SetTrajectoryJForceFz(fz=10)
    print("SetTrajectoryJForceFz return ", error)
    error = robot.SetTrajectoryJTorqueTx(tx=10)
    print("SetTrajectoryJTorqueTx return ", error)
    error = robot.SetTrajectoryJTorqueTy(ty=10)
    print("SetTrajectoryJTorqueTy return ", error)
    error = robot.SetTrajectoryJTorqueTz(tz=10)
    print("SetTrajectoryJTorqueTz return ", error)
    error = robot.LoadTrajectoryJ(name="/fruser/traj/test1011002.txt",ovl=30)
    print("LoadTrajectoryJ return ", error)
    error,startpos = robot.GetTrajectoryStartPose(name="/fruser/traj/test1011002.txt")
    print("Trajectory start pos is : ", startpos)
    robot.MoveCart(startpos, tool=0, user=0, vel=30)
    error,num = robot.GetTrajectoryPointNum()
    print("Trajectory return : ", error)
    robot.SetTrajectoryJSpeed(ovl=30)
    error = robot.MoveTrajectoryJ()
    print("MoveTrajectoryJ return ", error)


# settpd(robot)
# movetpd(robot)
loadtrajectoryj(robot)
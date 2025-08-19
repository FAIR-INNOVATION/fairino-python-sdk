from fairino import Robot
# A connection is established with the robot controller. A successful connection returns a robot object
# 与机器人控制器建立连接，连接成功返回一个机器人对象
robot = Robot.RPC('192.168.58.2')
import time

def TestExtAxisMoveBlend(self):
    # Define joint positions
    joint_pos1 = [-68.732, -99.773, -77.729, -77.167, 100.772, -13.317]
    joint_pos2 = [-101.678, -102.823, -77.512, -77.185, 88.388, -13.317]
    joint_pos3 = [-129.905, -99.715, -71.965, -77.209, 81.678, -13.317]
    # Define Cartesian positions
    desc_pos1 = [103.887, -434.739, 244.938, -162.495, 6.575, -142.948]
    desc_pos2 = [-196.883, -418.054, 218.942, -168.196, -4.388, -178.991]
    desc_pos3 = [-396.665, -265.695, 284.380, -160.913, -12.378, 149.770]
    # Define external axis positions
    epos1 = [0.000, 6.996, 0.000, 0.000]
    epos2 = [0.000, 20.987, 0.000, 0.000]
    epos3 = [-0.000, 30.982, 0.000, 0.000]
    offset_pos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    # Execute blended movement sequence
    rtn = robot.AccSmoothStart(saveFlag=True)
    print(f"AccSmoothStart rtn is {rtn}")
    time.sleep(1)
    rtn = robot.ExtAxisSyncMoveL(joint_pos1, desc_pos1, 1, 0, epos1, 100, 100, 100, 100, 0, 0, offset_pos)
    print(f"ExtAxisSyncMoveL 1 rtn is {rtn}")
    rtn = robot.ExtAxisSyncMoveL(joint_pos2, desc_pos2, 1, 0, epos2, 100, 100, 100, 200, 0, 0, offset_pos)
    print(f"ExtAxisSyncMoveL 2 rtn is {rtn}")
    rtn = robot.ExtAxisSyncMoveL(joint_pos3, desc_pos3, 1, 0, epos3, 100, 100, 100, 300, 0, 0, offset_pos)
    print(f"ExtAxisSyncMoveL 3 rtn is {rtn}")
    time.sleep(8)
    rtn = robot.AccSmoothEnd(saveFlag=True)
    print(f"AccSmoothEnd rtn is {rtn}")
    robot.CloseRPC()

def TestFirmWareUpgrade(self):
    robot.RobotEnable(0)
    time.sleep(0.2)  # 200ms
    rtn = robot.JointAllParamUpgrade("D://zUP/standardQX/jointallparametersFR56.0.db")
    print(f"robot JointAllParamUpgrade rtn is {rtn}")

    rtn = robot.SetCtrlFirmwareUpgrade(2, "D://zUP/standardQX/FAIR_Cobot_Cbd_Asix_V2.0.bin")
    print(f"robot SetCtrlFirmwareUpgrade rtn is {rtn}")
    rtn = robot.SetEndFirmwareUpgrade(2, "D://zUP/standardQX/FAIR_Cobot_Axle_Asix_V2.4.bin")
    print(f"robot SetEndFirmwareUpgrade rtn is {rtn}")

    robot.SetSysServoBootMode()
    time.sleep(0.2)  # 200ms

    rtn = robot.SetCtrlFirmwareUpgrade(1, "D://zUP/standardQX/FR_CTRL_PRIMCU_FV201011_MAIN_U4_T01_20250208.bin")
    print(f"robot SetCtrlFirmwareUpgrade rtn is {rtn}")
    rtn = robot.SetEndFirmwareUpgrade(1, "D://zUP/standardQX/FR_END_FV201008_MAIN_U01_T01_20250416.bin")
    print(f"robot SetEndFirmwareUpgrade rtn is {rtn}")
    rtn = robot.SetJointFirmwareUpgrade(1, "D://zUP/standardQX/FR_SERVO_FV502211_MAIN_U7_T07_20250217.bin")
    print(f"robot SetJointFirmwareUpgrade rtn is {rtn}")
    robot.CloseRPC()

def TestFirmWareUpgrade2(self):
    robot.RobotEnable(0)
    time.sleep(0.2)  # 200ms
    rtn = robot.JointAllParamUpgrade("D://zUP/standardQX/jointallparametersFR56.0.db")
    print(f"robot JointAllParamUpgrade rtn is {rtn}")

    rtn = robot.SetCtrlFirmwareUpgrade(2, "D://zUP/standardQX/FAIR_Cobot_Cbd_Asix_V2.0.bin")
    print(f"robot SetCtrlFirmwareUpgrade rtn is {rtn}")
    rtn = robot.SetEndFirmwareUpgrade(2, "D://zUP/standardQX/FAIR_Cobot_Axle_Asix_V2.4.bin")
    print(f"robot SetEndFirmwareUpgrade rtn is {rtn}")

    robot.SetSysServoBootMode()
    time.sleep(0.2)  # 200ms

    rtn = robot.SetCtrlFirmwareUpgrade(1, "D://zUP/standardQX/FR_CTRL_PRIMCU_FV201010_MAIN_U4_T01_20240529.bin")
    print(f"robot SetCtrlFirmwareUpgrade rtn is {rtn}")
    rtn = robot.SetEndFirmwareUpgrade(1, "D://zUP/standardQX/FR_END_FV201010_MAIN_U01_T01_20250522.bin")
    print(f"robot SetEndFirmwareUpgrade rtn is {rtn}")
    rtn = robot.SetJointFirmwareUpgrade(1, "D://zUP/standardQX/FR_SERVO_FV502211_MAIN_U7_T07_20250217.bin")
    print(f"robot SetJointFirmwareUpgrade rtn is {rtn}")
    robot.CloseRPC()

# TestExtAxisMoveBlend(robot)


# TestFirmWareUpgrade(robot)
TestFirmWareUpgrade2(robot)
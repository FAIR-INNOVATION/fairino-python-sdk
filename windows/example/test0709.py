from fairino import Robot
# A connection is established with the robot controller. A successful connection returns a robot object
import time
import threading

# 与机器人控制器建立连接，连接成功返回一个机器人对象
robot = Robot.RPC('192.168.58.2')

import time

import time


def test_firmware_upgrade(self):
    # Disable robot before upgrade
    # robot.SetSysServoBootMode()
    robot.RobotEnable(0)
    time.sleep(0.2)  # 200ms

    # Example firmware upgrade calls (commented out as in original)
    # rtn = robot.SetCtrlFirmwareUpgrade(1, "D://zUP/FR_CTRL_PRIMCU_FV201010_MAIN_U4_T01_20240529.bin")
    # print(f"robot SetCtrlFirmwareUpgrade rtn is {rtn}")
    # rtn = robot.SetEndFirmwareUpgrade(1, "D://zUP/FR_END_FV201008_MAIN_U01_T01_20250416.bin")
    # print(f"robot SetEndFirmwareUpgrade rtn is {rtn}")
    # rtn = robot.SetJointFirmwareUpgrade(1, "D://zUP/FR_SERVO_FV502211_MAIN_U7_T07_20250217.bin")
    # print(f"robot SetJointFirmwareUpgrade rtn is {rtn}")

    # Active joint parameters upgrade
    rtn = robot.JointAllParamUpgrade("D://zUP/jointallparametersFR56.0.db")
    print(f"robot JointAllParamUpgrade rtn is {rtn}")

    robot.CloseRPC()

def test_firmware_upgrade_mt(self):
    # robot.SetSysServoBootMode()
    # time.sleep(0.2)  # 200ms
    #
    # rtn = robot.SetCtrlFirmwareUpgrade(1, "D://zUP/MT/FR_CTRL_PRIMCU_FV201412_MAIN_U4_T01_20250630(MT).bin")
    # print(f"robot SetCtrlFirmwareUpgrade rtn is {rtn}")
    # rtn = robot.SetEndFirmwareUpgrade(1, "D://zUP/MT/FR_END_FV2010010_MAIN_U1_T01_20250603.bin")
    # print(f"robot SetEndFirmwareUpgrade rtn is {rtn}")
    # rtn = robot.SetJointFirmwareUpgrade(1, "D://zUP/MT/FR_SERVO_FV504215_MAIN_U7_T07_20250603.bin")
    # print(f"robot SetJointFirmwareUpgrade rtn is {rtn}")
    # robot.CloseRPC()

    robot.SetSysServoBootMode()
    time.sleep(0.2)  # 200ms

    rtn = robot.SetCtrlFirmwareUpgrade(2, "D://zUP/MT/FAIR_Cobot_Cbd_Asix_V2.0.bin")
    print(f"robot SetCtrlFirmwareUpgrade rtn is {rtn}")
    rtn = robot.SetEndFirmwareUpgrade(2, "D://zUP/MT/FAIR_Cobot_Axle_Asix_V2.4.bin")
    print(f"robot SetEndFirmwareUpgrade rtn is {rtn}")
    robot.CloseRPC()

    # robot.RobotEnable(0)
    # time.sleep(0.2)  # 200ms
    # rtn = robot.JointAllParamUpgrade("D://zUP/MT/joint0603/jointallparameters.db")
    # print(f"robot JointAllParamUpgrade rtn is {rtn}")

    robot.CloseRPC()

def TestFirmWareUpgrade(self):
    robot.RobotEnable(0)
    time.sleep(0.2)  # 200ms
    rtn = robot.JointAllParamUpgrade("D://zUP/MT/joint0603/jointallparameters.db")
    print(f"robot JointAllParamUpgrade rtn is {rtn}")

    rtn = robot.SetCtrlFirmwareUpgrade(2, "D://zUP/MT/FAIR_Cobot_Cbd_Asix_V2.0.bin")
    print(f"robot SetCtrlFirmwareUpgrade rtn is {rtn}")
    rtn = robot.SetEndFirmwareUpgrade(2, "D://zUP/MT/FAIR_Cobot_Axle_Asix_V2.4.bin")
    print(f"robot SetEndFirmwareUpgrade rtn is {rtn}")

    robot.SetSysServoBootMode()
    time.sleep(0.2)  # 200ms

    rtn = robot.SetCtrlFirmwareUpgrade(1, "D://zUP/MT/FR_CTRL_PRIMCU_FV201412_MAIN_U4_T01_20250630(MT).bin")
    print(f"robot SetCtrlFirmwareUpgrade rtn is {rtn}")
    rtn = robot.SetEndFirmwareUpgrade(1, "D://zUP/MT/FR_END_FV2010010_MAIN_U1_T01_20250603.bin")
    print(f"robot SetEndFirmwareUpgrade rtn is {rtn}")
    rtn = robot.SetJointFirmwareUpgrade(1, "D://zUP/MT/FR_SERVO_FV504215_MAIN_U7_T07_20250603.bin")
    print(f"robot SetJointFirmwareUpgrade rtn is {rtn}")
    robot.CloseRPC()

def test(self):
    error,tcp = robot.GetActualTCPPose()
    print(tcp[0])



# test_firmware_upgrade(robot)
# test_firmware_upgrade_mt(robot)
# TestFirmWareUpgrade(robot)
test(robot)
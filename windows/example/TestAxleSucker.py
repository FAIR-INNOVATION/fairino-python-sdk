from fairino import Robot
import time


def main():
    # Establish connection with the robot controller
    robot = Robot.RPC('192.168.58.2')
    time.sleep(0.5)  # Wait for connection and data reception

    print("========== End Sucker Interface Test ==========")

    # Define motion points
    j1 = [168.610, -49.414, 19.171, -59.520, -90.466, -9.516]
    j2 = [168.610, -53.606, 31.351, -67.508, -90.466, -9.516]
    d1 = [725.139, -42.866, 573.293, 179.690, -0.420, -91.872]
    d2 = [725.147, -42.866, 543.279, 179.689, -0.421, -91.872]
    ex = [0.0, 0.0, 0.0, 0.0]
    zeroOff = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    id = 1  # Slave station number

    # ========== Enable end device: enable sucker and read back ==========
    rtn, forceEn, gripEn, ioEn, dexEn, suckEn = robot.GetAxleLuaEnableDeviceType()
    print(f"GetAxleLuaEnableDeviceType rtn={rtn}, force={forceEn}, gripper={gripEn}, io={ioEn}, dexhand={dexEn}, sucker={suckEn}")

    rtn = robot.SetAxleLuaEnableDeviceType(forceEn, gripEn, ioEn, dexEn, 1)
    print(f"SetAxleLuaEnableDeviceType(sucker=1): {rtn}")

    rtn, forceEn, gripEn, ioEn, dexEn, suckEn = robot.GetAxleLuaEnableDeviceType()
    print(f"GetAxleLuaEnableDeviceType(after set) rtn={rtn}, sucker={suckEn}")

    # ========== Get enable device arrays ==========
    rtn, forceArr, gripperArr, ioArr, dexArr, suckArr = robot.GetAxleLuaEnableDevice()
    print(f"GetAxleLuaEnableDevice rtn={rtn}, sucker={suckArr}")

    # ========== Set sucker function codes ==========
    # func[0]~func[12] correspond to Bit0~Bit12, all enabled
    # func[11]=Bit11-channel1 control enable, func[12]=Bit12-channel2 control enable
    suckerFunc = [0] * 16
    for i in range(13):
        suckerFunc[i] = 1

    rtn = robot.SetAxleSuckerFunc(id, suckerFunc)
    print(f"SetAxleSuckerFunc: {rtn}")

    # Read back 16 function codes
    rtn, funcValue = robot.GetAxleSuckerFunc(id)
    print(f"GetAxleSuckerFunc rtn={rtn}, value={funcValue}")

    # ========== Reset and activate sucker ==========
    rtn = robot.SetAxleSuckerAct(id, 0)  # 0-reset
    print(f"SetAxleSuckerAct(0): {rtn}")

    rtn = robot.SetAxleSuckerAct(id, 1)  # 1-activate
    print(f"SetAxleSuckerAct(1): {rtn}")

    time.sleep(2)

    # ========== Move to suction position ==========
    rtn = robot.MoveL(desc_pos=d2, tool=0, user=0, joint_pos=j2,
                      vel=100, acc=100, ovl=100, blendR=-1, blendMode=0,
                      exaxis_pos=ex, search=0, offset_flag=0, offset_pos=zeroOff,
                      oacc=100, config=0, velAccParamMode=0, overSpeedStrategy=0, speedPercent=10)
    print(f"moveL errcode:{rtn}")

    # ========== Set sucker control parameters ==========
    channel = [1, 1, 0, 0]     # 1-enable, 0-disable
    minPress = [60, 60, 0, 0]  # Minimum vacuum pressure
    maxPress = [70, 70, 0, 0]  # Maximum vacuum pressure
    rtn = robot.SetAxleSuckerCtrl(id, channel, minPress, maxPress, 6000)
    print(f"SetAxleSuckerCtrl: {rtn}")

    # Wait channel 1 (channel=0) to pick object
    rtn = robot.WaitAxleSuckerState(0, 6000, 1, 0)
    print(f"WaitAxleSuckerState ch0 pick: {rtn}")

    # Wait channel 2 (channel=1) to pick object
    rtn = robot.WaitAxleSuckerState(1, 6000, 1, 0)
    print(f"WaitAxleSuckerState ch1 pick: {rtn}")

    # ========== Query sucker state for 10 cycles ==========
    count = 10
    for i in range(count):
        # channel 0~3 correspond to channels 1~4, print first two channels
        for ch in range(2):
            rtn, enable, done, vacuumValue, error = robot.GetAxleSuckerState(ch)
            print(f"[{i + 1}/{count}] Channel{ch + 1}(channel={ch}) rtn={rtn}, "
                  f"enable={enable}, done={done}, vacuumValue={vacuumValue}, error={error}")
        time.sleep(0.5)
        print(f"00000: {rtn}")

    print(f"11111111: {rtn}")

    # ========== Move to lift position ==========
    rtn = robot.MoveL(desc_pos=d1, tool=0, user=0, joint_pos=j1,
                      vel=100, acc=100, ovl=100, blendR=-1, blendMode=0,
                      exaxis_pos=ex, search=0, offset_flag=0, offset_pos=zeroOff,
                      oacc=100, config=0, velAccParamMode=0, overSpeedStrategy=0, speedPercent=10)
    print(f"moveL errcode:{rtn}")

    # ========== Close all channels after waiting ==========
    channel = [0, 0, 0, 0]
    rtn = robot.SetAxleSuckerCtrl(id, channel, minPress, maxPress, 8000)
    print(f"SetAxleSuckerCtrl(close channels): {rtn}")

    # Wait channel 1 (channel=0) to release object
    rtn = robot.WaitAxleSuckerState(0, 6000, 0, 0)
    print(f"WaitAxleSuckerState ch0 release: {rtn}")

    # Wait channel 2 (channel=1) to release object
    rtn = robot.WaitAxleSuckerState(1, 6000, 0, 0)
    print(f"WaitAxleSuckerState ch1 release: {rtn}")

    # Close connection
    robot.CloseRPC()


if __name__ == "__main__":
    main()
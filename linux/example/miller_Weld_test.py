from fairino import Robot
import time

from fairino.Robot import AddRobotRealtimeState, RobotState


def test_weld_set_param(robot):
    """Weld machine set parameter test (observe PLC D242~D249, D257.2)"""
    print("\n========== Weld Set Param Test ==========")

    j1 = [-143.902, -59.161, 101.339, -130.844, -90.116, -56.026]
    j2 = [-112.507, -77.118, 129.920, -141.195, -89.783, -24.639]
    d1 = [436.675, 437.436, 148.030, 178.907, 0.265, 1.550]
    d2 = [75.368, 437.428, 148.031, 178.907, 0.265, 1.549]
    ex = [0.0, 0.0, 0.0, 0.0]
    zeroOff = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    # 1) Select arc welding + Miller welder; io_type=1 means UDP link
    rtn = robot.SetWeldType(0)  # 0-arc welding, 1-laser welding
    print(f"SetWeldType(arc): {rtn}")
    rtn = robot.SetWeldCompany(104)  # 104-Miller
    print(f"SetWeldCompany(Miller): {rtn}")
    rtn = robot.ExtDevLoadUDPDriver()  # Establish robot UDP peripheral communication
    print(f"ExtDevLoadUDPDriver: {rtn}")
    time.sleep(2)

    rtn = robot.MoveL(desc_pos=d1, tool=0, user=0, joint_pos=j1,
                      vel=100, acc=100, ovl=100, blendR=-1, blendMode=0,
                      exaxis_pos=ex, search=0, offset_flag=0, offset_pos=zeroOff,
                      oacc=100, config=0, velAccParamMode=0, overSpeedStrategy=0, speedPercent=10)
    print(f"moveL errcode:{rtn}")

    # 2) Default control words 44~47 -> D242~D245
    rtn = robot.SetweldWFSUdpNum(44)
    print(f"SetweldWFSUdpNum(44): {rtn}")
    rtn = robot.SetWeldArcLenUdpNum(45)
    print(f"SetWeldArcLenUdpNum(45): {rtn}")
    rtn = robot.SetWeldArcCtrlUdpNum(46)
    print(f"SetWeldArcCtrlUdpNum(46): {rtn}")
    rtn = robot.SetWeldSlotUdpNum(47)
    print(f"SetWeldSlotUdpNum(47): {rtn}")
    time.sleep(3)

    rtn = robot.SetWeldWFS(1, 300.0)  # Wire feed speed 300
    print(f"SetWeldWFS(300): {rtn}")
    rtn = robot.SetWeldArcLen(1, 0)  # Arc length 0 (500 is middle value)
    print(f"SetWeldArcLen(0): {rtn}")
    rtn = robot.SetWeldArcCtrl(1, 25)  # Arc control 25 (default)
    print(f"SetWeldArcCtrl(25): {rtn}")
    rtn = robot.SetWeldSlot(1, 1)  # Program slot 1
    print(f"SetWeldSlot(1): {rtn}")
    print(">>> Please confirm PLC D242~D245 updated to 300/0/25/1")
    time.sleep(3)

    rtn = robot.MoveL(desc_pos=d2, tool=0, user=0, joint_pos=j2,
                      vel=100, acc=100, ovl=100, blendR=-1, blendMode=0,
                      exaxis_pos=ex, search=0, offset_flag=0, offset_pos=zeroOff,
                      oacc=100, config=0, velAccParamMode=0, overSpeedStrategy=0, speedPercent=10)
    print(f"moveL errcode:{rtn}")

    # 3) Custom control words 48~51 -> D246~D249
    rtn = robot.SetweldWFSUdpNum(48)
    print(f"SetweldWFSUdpNum(48): {rtn}")
    rtn = robot.SetWeldArcLenUdpNum(49)
    print(f"SetWeldArcLenUdpNum(49): {rtn}")
    rtn = robot.SetWeldArcCtrlUdpNum(50)
    print(f"SetWeldArcCtrlUdpNum(50): {rtn}")
    rtn = robot.SetWeldSlotUdpNum(51)
    print(f"SetWeldSlotUdpNum(51): {rtn}")
    time.sleep(3)

    # 4) Write same control words (48~51) for 10 cycles, each value +1 per round
    for i in range(1, 11):
        wfs = 350.0 + i
        arcLen = 10 + i
        arcCtrl = 30 + i
        slot = 1 + (i - 1) % 8  # Slot range 1~8, wrap around

        rtn = robot.SetWeldWFS(1, wfs)
        rtn = robot.SetWeldArcLen(1, arcLen)
        rtn = robot.SetWeldArcCtrl(1, arcCtrl)
        rtn = robot.SetWeldSlot(1, slot)

        print(f">>> [{i}/10] D246~D249 should update to {wfs}/{arcLen}/{arcCtrl}/{slot}")
        time.sleep(0.5)

    rtn = robot.MoveL(desc_pos=d1, tool=0, user=0, joint_pos=j1,
                      vel=100, acc=100, ovl=100, blendR=-1, blendMode=0,
                      exaxis_pos=ex, search=0, offset_flag=0, offset_pos=zeroOff,
                      oacc=100, config=0, velAccParamMode=0, overSpeedStrategy=0, speedPercent=10)
    print(f"moveL errcode:{rtn}")

    # 5) Disable welding: extended DO default 18 -> D257.2
    rtn = robot.SetWeldDisableExtDoNum(18)
    print(f"SetWeldDisableExtDoNum(18): {rtn}")
    rtn = robot.SetWeldDisable(1, 1)
    print(f"SetWeldDisable(1): {rtn}")
    print(">>> Please confirm PLC D257.2 is set to 1, auto restore to 0 after 5 seconds")
    time.sleep(1.5)
    rtn = robot.SetWeldDisable(1, 0)
    print(f"SetWeldDisable(0): {rtn}")

    rtn = robot.MoveL(desc_pos=d2, tool=0, user=0, joint_pos=j2,
                      vel=100, acc=100, ovl=100, blendR=-1, blendMode=0,
                      exaxis_pos=ex, search=0, offset_flag=0, offset_pos=zeroOff,
                      oacc=100, config=0, velAccParamMode=0, overSpeedStrategy=0, speedPercent=10)
    print(f"moveL errcode:{rtn}")


def test_weld_feedback(robot):
    """Weld machine feedback print test"""
    print("\n========== Weld Feedback Test ==========")

    rtn = robot.SetWeldType(0)  # 0-arc welding, 1-laser welding
    print(f"SetWeldType(arc): {rtn}")
    rtn = robot.SetWeldCompany(104)  # 104-Miller
    print(f"SetWeldCompany(Miller): {rtn}")
    rtn = robot.ExtDevLoadUDPDriver()  # Establish robot UDP peripheral communication
    print(f"ExtDevLoadUDPDriver: {rtn}")
    rtn = robot.SetWeldSynStateExtDINum(0)  # Set sync state DI
    print(f"SetWeldSynStateExtDINum: {rtn}")
    rtn = robot.SetWeldWfsFeedbackUdpNum(47)  # Set wire feed speed feedback UDP status word number
    print(f"SetWeldWfsFeedbackUdpNum(47): {rtn}")
    time.sleep(3)

    count = 1800
    for i in range(count):
        # Get robot real-time state (voltage/current)
        stateRtn, pkg = robot.GetRobotRealTimeState()

        # Get wire feed speed feedback (io_type=1 UDP)
        wfsRtn, wfs = robot.GetWeldWfsFeedback(1)

        # Get sync state (io_type=1 UDP)
        synRtn, synState = robot.GetWeldSynState(1)
        # Get robot error code
        errRtn, errorcode = robot.GetRobotErrorCode()

        print(f"[{i}] Voltage={pkg.weldVoltage:.2f}V Current={pkg.weldCurrent:.2f}A "
              f"WFS={wfs:.2f}(rtn={wfsRtn}) Sync={synState}(rtn={synRtn}) "
              f"MainErr={errorcode[0]} SubErr={errorcode[1]}(rtn={errRtn}) stateRtn={stateRtn}")
        print(f"pkg.jt_cur_pos[5]: {pkg.jt_cur_pos[5]}")
        time.sleep(0.5)


def main():
    # Establish connection with the robot controller
    rtn = AddRobotRealtimeState([
        RobotState.WeldVoltage,
        RobotState.WeldCurrent,
    ])

    robot = Robot.RPC('192.168.58.2')
    time.sleep(0.5)  # Wait for connection and data reception

    # Run the desired test (uncomment one at a time)
    # test_weld_set_param(robot)
    test_weld_feedback(robot)
    # Close connection
    robot.CloseRPC()


if __name__ == "__main__":
    main()
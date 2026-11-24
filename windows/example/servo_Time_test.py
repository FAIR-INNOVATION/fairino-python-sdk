from fairino import Robot
import time


def udp_frame_callback(src_type, count, cmd_id, data_len, content):
    """UDP frame receive callback"""
    print("Callback: cmd_id={} count={} data_len={} content={}".format(cmd_id, count, data_len, content))
    return 0


def test_servo_j_with_filtertime(robot):
    """ServoJ with filter time and servo gain test"""
    print("\n========== ServoJ with Filter Time Test ==========")

    j = [70.000000, -49.000000, -109.000000, -81.000000, 76.000000, 93.000000]
    epos = [0.0, 0.0, 0.0, 0.0]
    offset_pos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    # Move to initial joint position
    robot.MoveJ(joint_pos=j, tool=0, user=0, vel=100, acc=100, ovl=100,
                exaxis_pos=epos, blendT=-1, offset_flag=0, offset_pos=offset_pos)

    vel = 0.0
    acc = 0.0
    cmdT = 0.008
    filterT = 0.0
    gain = 0.0
    cmdID = 0
    comType = 0
    filtertime = 0
    ServoGain = [22.0, 22.0, 22.0, 22.0, 22.0, 22.0]

    # Get current joint position
    rtn, j = robot.GetActualJointPosDegree(0)
    if rtn != 0:
        print(f"GetActualJointPosDegree errcode:{rtn}")

    # Start servo motion with filter time and gain
    rtn = robot.ServoMoveStart(comType, filtertime, ServoGain)
    print(f"ServoMoveStart rtn is {rtn}")

    # Read trajectory file and send ServoJ commands
    file_path = "D://zUP/6RPTPpath.txt"
    try:
        with open(file_path, 'r') as fin:
            for line in fin:
                line = line.strip()
                if not line:
                    continue
                # Parse line format: ServoJ(j1,j2,j3,j4,j5,j6,...
                if line.startswith("ServoJ("):
                    values_str = line[len("ServoJ("):]
                    parts = values_str.split(',')
                    if len(parts) >= 6:
                        p = [float(parts[i]) for i in range(6)]
                        rtn = robot.ServoJ(joint_pos=p, axisPos=epos, acc=acc, vel=vel,
                                           cmdT=cmdT, filterT=filterT, gain=gain,
                                           id=cmdID, cmdType=comType)
                        print(f"ServoJ rtn is {rtn} , line  {line}")
    except FileNotFoundError:
        print(f"File not found: {file_path}")
    print("111111111111")
    # End servo motion
    rtn = robot.ServoMoveEnd(comType)
    print(f"ServoMoveEnd rtn is {rtn}")

    time.sleep(4)


def test_servo_cart_with_filtertime(robot):
    """ServoCart with filter time and servo gain test"""
    print("\n========== ServoCart with Filter Time Test ==========")

    desc_pos_dt = [542.1690, 34.7990, 114.4800, 169.808000, -3.600000, -133.152000]
    exaxis = [0.0, 0.0, 0.0, 0.0]
    pos_gain = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    mode = 0
    vel = 0.0
    acc = 0.0
    cmdT = 0.008
    filterT = 0.0
    gain = 0.0
    filtertime = 80
    ServoGain = [20.0, 20.0, 20.0, 20.0, 20.0, 20.0]

    # Set speed and move to start point
    robot.SetSpeed(20)
    offset_pos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    robot.MoveL(desc_pos=desc_pos_dt, tool=1, user=0, vel=100, acc=100, ovl=100,
                blendR=-1, blendMode=0, exaxis_pos=exaxis, search=0,
                offset_flag=0, offset_pos=offset_pos)

    # Start servo motion with filter time and gain
    rtn = robot.ServoMoveStart(0, filtertime, ServoGain)
    print(f"ServoMoveStart rtn is {rtn}")

    # Read trajectory file and send ServoCart commands
    file_path = "D://zUP/6RCARpath.txt"
    try:
        with open(file_path, 'r') as fin:
            for line in fin:
                line = line.strip()
                if not line:
                    continue
                # Parse line format: ServoCart(0,x,y,z,rx,ry,rz,...
                if line.startswith("ServoCart("):
                    values_str = line[len("ServoCart("):]
                    parts = values_str.split(',')
                    # Skip the first element (mode=0), take next 6 values
                    if len(parts) >= 7:
                        desc_pos_dt = [float(parts[i]) for i in range(1, 7)]
                        rtn = robot.ServoCart(mode=mode, desc_pos=desc_pos_dt, exaxis=exaxis,
                                              pos_gain=pos_gain, acc=acc, vel=vel,
                                              cmdT=cmdT, filterT=filterT, gain=gain)
                        print(f"ServoCart rtn is {rtn}")
    except FileNotFoundError:
        print(f"File not found: {file_path}")

    # End servo motion
    robot.ServoMoveEnd()


def main():
    # Establish connection with the robot controller
    robot = Robot.RPC('192.168.58.5')
    time.sleep(0.5)  # Wait for connection and data reception

    # Set UDP command reply callback
    robot.SetUDPCmdRpyCallback(udp_frame_callback)

    # Run the desired test (uncomment one at a time)
    test_servo_j_with_filtertime(robot)
    # test_servo_cart_with_filtertime(robot)

    # Close connection
    robot.CloseRPC()


if __name__ == "__main__":
    main()
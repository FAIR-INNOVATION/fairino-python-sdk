from fairino import Robot
import time


def test_moveL_speed_loop(robot):
    """MoveL back and forth with speed 20->30->40"""
    print("\n========== MoveL Speed Loop Test ==========")

    # MoveL point 1: joint + desc
    pos1 = [52.055, -53.917, 83.053, -119.137, -90.000, -40.315]
    pose1 = [-348.348, -612.635, 203.147, -180.000, -0.000, -177.630]

    # MoveL point 2: joint + desc
    pos2 = [-51.966, -68.141, 106.161, -128.021, -90.000, -144.338]
    pose2 = [-432.405, 387.234, 203.149, 179.999, -0.000, -177.628]

    epos = [0.0, 0.0, 0.0, 0.0]
    offdese = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    # SetSpeed(20)
    rtn = robot.SetSpeed(20)
    print(f"SetSpeed(20): {rtn}")

    # Initial MoveJ to point 1
    rtn = robot.MoveJ(joint_pos=pos1, desc_pos=pose1, tool=8, user=0,
                      vel=100, acc=100, ovl=100, exaxis_pos=epos,
                      blendT=-1, offset_flag=0, offset_pos=offdese)
    print(f"MoveJ pos1: {rtn}")

    # Infinite loop: MoveL back and forth with speed 20->30->40
    while True:
        # MoveL to point 2
        rtn = robot.MoveL(desc_pos=pose2, tool=8, user=0, joint_pos=pos2,
                          vel=100, acc=100, ovl=100, blendR=-1, blendMode=0,
                          exaxis_pos=epos, search=0, offset_flag=0, offset_pos=offdese,
                          oacc=100, config=-1, velAccParamMode=0, overSpeedStrategy=0, speedPercent=10)
        print(f"MoveL pos2: {rtn}")

        # SetSpeed(30)
        rtn = robot.SetSpeed(30)
        print(f"SetSpeed(30): {rtn}")

        # MoveL back to point 1
        rtn = robot.MoveL(desc_pos=pose1, tool=8, user=0, joint_pos=pos1,
                          vel=100, acc=100, ovl=100, blendR=-1, blendMode=0,
                          exaxis_pos=epos, search=0, offset_flag=0, offset_pos=offdese,
                          oacc=100, config=-1, velAccParamMode=0, overSpeedStrategy=0, speedPercent=10)
        print(f"MoveL pos1: {rtn}")

        # SetSpeed(40)
        rtn = robot.SetSpeed(40)
        print(f"SetSpeed(40): {rtn}")


def test_moveC_speed_loop(robot):
    """MoveC back and forth with speed 20->30->40"""
    print("\n========== MoveC Speed Loop Test ==========")

    # MoveC start point
    pos_start = [-90.489, -84.415, 127.849, -133.531, -90.000, -74.001]
    pose_start = [-98.324, 431.220, 203.588, -179.974, -0.093, 73.512]

    # MoveC mid point
    pos_mid = [-108.333, -61.555, 95.797, -124.336, -89.971, -91.845]
    pose_mid = [101.649, 631.196, 203.595, -179.974, -0.094, 73.512]

    # MoveC end point 1 (returns to start)
    pos_end1 = [-90.489, -84.415, 127.849, -133.531, -90.000, -74.001]
    pose_end1 = [-98.324, 431.220, 203.588, -179.974, -0.093, 73.512]

    # MoveC end point 2
    pos_end2 = [-90.253, -35.630, 49.719, -104.188, -90.001, -73.765]
    pose_end2 = [-98.326, 831.170, 203.600, -179.973, -0.094, 73.512]

    epos = [0.0, 0.0, 0.0, 0.0]
    offdese = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    # SetSpeed(20)
    rtn = robot.SetSpeed(20)
    print(f"SetSpeed(20): {rtn}")

    # Initial MoveJ to start point
    rtn = robot.MoveJ(joint_pos=pos_start, desc_pos=pose_start, tool=8, user=0,
                      vel=100, acc=100, ovl=100, exaxis_pos=epos,
                      blendT=-1, offset_flag=0, offset_pos=offdese)
    print(f"MoveJ start: {rtn}")

    # Infinite loop: MoveC with speed 20->30->40
    while True:
        # MoveC: mid -> end1 (returns to start)
        rtn = robot.MoveC(desc_pos_p=pose_mid, tool_p=8, user_p=0,
                          desc_pos_t=pose_end2, tool_t=8, user_t=0,
                          joint_pos_p=pos_mid, joint_pos_t=pos_end2,
                          vel_p=100, acc_p=100, exaxis_pos_p=epos, offset_flag_p=0, offset_pos_p=offdese,
                          vel_t=100, acc_t=100, exaxis_pos_t=epos, offset_flag_t=0, offset_pos_t=offdese,
                          ovl=100, blendR=-1, oacc=100, config=-1, velAccParamMode=0)
        print(f"MoveC (mid->end1): {rtn}")

        # SetSpeed(30)
        rtn = robot.SetSpeed(30)
        print(f"SetSpeed(30): {rtn}")

        # MoveC: mid -> end2
        rtn = robot.MoveC(desc_pos_p=pose_mid, tool_p=8, user_p=0,
                          desc_pos_t=pose_end1, tool_t=8, user_t=0,
                          joint_pos_p=pos_mid, joint_pos_t=pos_end1,
                          vel_p=100, acc_p=100, exaxis_pos_p=epos, offset_flag_p=0, offset_pos_p=offdese,
                          vel_t=100, acc_t=100, exaxis_pos_t=epos, offset_flag_t=0, offset_pos_t=offdese,
                          ovl=100, blendR=-1, oacc=100, config=-1, velAccParamMode=0)
        print(f"MoveC (mid->end2): {rtn}")

        # SetSpeed(40)
        rtn = robot.SetSpeed(40)
        print(f"SetSpeed(40): {rtn}")


def test_circle_speed_loop(robot):
    """Circle back and forth with speed 20->30->40"""
    print("\n========== Circle Speed Loop Test ==========")

    # Circle start point
    pos_start = [-90.489, -84.415, 127.849, -133.531, -90.000, -74.001]
    pose_start = [-98.324, 431.220, 203.588, -179.974, -0.093, 73.512]

    # Circle mid point
    pos_mid = [-108.333, -61.555, 95.797, -124.336, -89.971, -91.845]
    pose_mid = [101.649, 631.196, 203.595, -179.974, -0.094, 73.512]

    # Circle end point
    pos_end = [-90.253, -35.630, 49.719, -104.188, -90.001, -73.765]
    pose_end = [-98.326, 831.170, 203.600, -179.973, -0.094, 73.512]

    epos = [0.0, 0.0, 0.0, 0.0]
    offdese = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    # SetSpeed(20)
    rtn = robot.SetSpeed(20)
    print(f"SetSpeed(20): {rtn}")

    # Initial MoveJ to start point
    rtn = robot.MoveJ(joint_pos=pos_start, desc_pos=pose_start, tool=8, user=0,
                      vel=100, acc=100, ovl=100, exaxis_pos=epos,
                      blendT=-1, offset_flag=0, offset_pos=offdese)
    print(f"MoveJ start: {rtn}")

    # Infinite loop: Circle with speed 20->30->40
    while True:
        # SetSpeed(30)
        rtn = robot.SetSpeed(30)
        print(f"SetSpeed(30): {rtn}")

        # Circle: mid -> end
        rtn = robot.Circle(desc_pos_p=pose_mid, tool_p=8, user_p=0,
                           desc_pos_t=pose_end, tool_t=8, user_t=0,
                           joint_pos_p=pos_mid, joint_pos_t=pos_end,
                           vel_p=100, acc_p=100, exaxis_pos_p=epos,
                           vel_t=100, acc_t=100, exaxis_pos_t=epos,
                           ovl=100, offset_flag=0, offset_pos=offdese,
                           oacc=100, blendR=-1, config=-1, velAccParamMode=0)
        print(f"Circle (mid->end): {rtn}")

        # SetSpeed(40)
        rtn = robot.SetSpeed(40)
        print(f"SetSpeed(40): {rtn}")


def main():
    # Establish connection with the robot controller
    robot = Robot.RPC('192.168.58.2')
    time.sleep(0.5)  # Wait for connection and data reception

    # Uncomment the test you want to run
    # test_moveL_speed_loop(robot)
    # test_moveC_speed_loop(robot)
    test_circle_speed_loop(robot)

    # Close connection
    robot.CloseRPC()


if __name__ == "__main__":
    main()
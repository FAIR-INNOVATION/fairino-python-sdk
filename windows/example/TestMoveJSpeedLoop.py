from fairino import Robot
import time


def main():
    # Establish connection with the robot controller
    robot = Robot.RPC('192.168.58.2')
    time.sleep(0.5)  # Wait for connection and data reception

    # Lua point 1 joint
    pos1 = [23.424, -76.529, 114.134, -113.992, 46.783, -69.413]
    # Lua point 1 cartesian
    pose1 = [-416.922, -366.417, 371.091, -37.336, 27.024, -139.857]
    # Lua point 2 (loop) joint: only j5 differs = -153.191
    pos2 = [23.424, -76.529, 114.134, -113.992, -153.191, -69.413]
    # Lua point 2 (loop) cartesian
    pose2 = [-454.146, -210.648, 256.427, 116.099, -4.858, -165.734]
    offdese = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    epos = [0.0, 0.0, 0.0, 0.0]

    # SetSpeed(20)
    rtn = robot.SetSpeed(20)
    print(f"SetSpeed(20): {rtn}")

    # Initial MoveJ to point 1 (tool=8, user=0, vel/acc/ovl=100, blendT=-1, no offset)
    rtn = robot.MoveJ(joint_pos=pos1, desc_pos=pose1, tool=8, user=0,
                      vel=100, acc=100, ovl=100, exaxis_pos=epos,
                      blendT=-1, offset_flag=0, offset_pos=offdese)
    print(f"MoveJ pos1: {rtn}")

    # Infinite loop: back-and-forth motion with speed increasing 20->30->40
    while True:
        # MoveJ to point 2
        rtn = robot.MoveJ(joint_pos=pos2, desc_pos=pose2, tool=8, user=0,
                          vel=100, acc=100, ovl=100, exaxis_pos=epos,
                          blendT=-1, offset_flag=0, offset_pos=offdese)
        print(f"MoveJ pos2: {rtn}")

        # SetSpeed(30)
        rtn = robot.SetSpeed(30)
        print(f"SetSpeed(30): {rtn}")

        # MoveJ back to point 1
        rtn = robot.MoveJ(joint_pos=pos1, desc_pos=pose1, tool=8, user=0,
                          vel=100, acc=100, ovl=100, exaxis_pos=epos,
                          blendT=-1, offset_flag=0, offset_pos=offdese)
        print(f"MoveJ pos1: {rtn}")

        # SetSpeed(40)
        rtn = robot.SetSpeed(40)
        print(f"SetSpeed(40): {rtn}")

    # Close connection (never reached due to infinite loop)
    robot.CloseRPC()


if __name__ == "__main__":
    main()
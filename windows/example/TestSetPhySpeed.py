from fairino import Robot
import time
import threading


def robot_thread_func(robot):
    """Background thread: read user input and set speed instantly"""
    while True:
        try:
            input_val = input("Please input speed value: ")
            input_val = float(input_val)
            rtn = robot.SetPhySpeedInstant(input_val)
            print(f"SetPhySpeedInstant({input_val}) rtn is {rtn}")
        except ValueError:
            print("Invalid number!")


def main():
    # Establish connection with the robot controller
    robot = Robot.RPC('192.168.58.2')
    time.sleep(0.5)  # Wait for connection and data reception

    # Start background thread for speed input
    th = threading.Thread(target=robot_thread_func, args=(robot,))
    th.daemon = True
    th.start()

    # Define motion points
    j1 = [-60.206, -49.449, 79.476, -124.322, -88.416, -45.209]
    d1 = [-412.929, 510.912, 94.557, -175.850, -1.933, 74.992]
    j2 = [-141.282, -40.962, 62.359, -110.496, -85.512, -126.379]
    d2 = [586.847, 510.915, 94.560, -175.851, -1.934, 74.991]

    ex = [0.0, 0.0, 0.0, 0.0]
    zeroOff = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    # Infinite loop: MoveL back and forth between two points
    while True:
        rtn = robot.MoveL(desc_pos=d1, tool=1, user=0, joint_pos=j1,
                          vel=100, acc=100, ovl=100, blendR=-1, blendMode=0,
                          exaxis_pos=ex, search=0, offset_flag=0, offset_pos=zeroOff,
                          oacc=100, config=0, velAccParamMode=0, overSpeedStrategy=0, speedPercent=10)
        print(f"MoveL j1->d1 rtn: {rtn}")

        rtn = robot.MoveL(desc_pos=d2, tool=1, user=0, joint_pos=j2,
                          vel=100, acc=100, ovl=100, blendR=-1, blendMode=0,
                          exaxis_pos=ex, search=0, offset_flag=0, offset_pos=zeroOff,
                          oacc=100, config=0, velAccParamMode=0, overSpeedStrategy=0, speedPercent=10)
        print(f"MoveL j2->d2 rtn: {rtn}")

    # Close connection (never reached due to infinite loop)
    robot.CloseRPC()


if __name__ == "__main__":
    main()
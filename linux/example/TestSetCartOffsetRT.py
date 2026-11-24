from fairino import Robot
import time
import threading
import sys


def cart_offset_pos(robot):
    """Background thread: read user input and set cartesian offset in real time"""
    while True:
        try:
            # Print prompt on a new line to avoid mixing with motion logs
            sys.stdout.write("\nPlease input offset y value: ")
            sys.stdout.flush()
            input_val = input()
            input_val = float(input_val)
            # Build offset pose with only y-axis offset
            offset_pos = [0.0, input_val, 0.0, 0.0, 0.0, 0.0]
            rtn = robot.SetCartOffsetRT(2, offset_pos)
            print(f"\nSetCartOffsetRT(2, y={input_val}) rtn is {rtn}")
        except ValueError:
            print("\nInvalid number!")


def main():
    # Establish connection with the robot controller
    robot = Robot.RPC('192.168.58.2')
    time.sleep(0.5)  # Wait for connection and data reception

    # Start background thread for offset input
    th = threading.Thread(target=cart_offset_pos, args=(robot,))
    th.daemon = True
    th.start()

    # Define motion points
    j1 = [-72.561, -38.933, 92.047, -143.101, -90.000, 17.439]
    j2 = [-100.211, -44.358, 105.565, -151.192, -90.007, -10.211]
    d1 = [-300.008, 599.999, -199.999, -0.013, -0.004, 0.000]
    d2 = [-0.004, 599.977, -200.038, -0.017, -0.004, 0.000]

    ex = [0.0, 0.0, 0.0, 0.0]
    zeroOff = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    # Infinite loop: MoveL back and forth between two points
    while True:
        rtn = robot.MoveL(desc_pos=d1, tool=1, user=0, joint_pos=j1,
                          vel=100, acc=100, ovl=100, blendR=-1, blendMode=0,
                          exaxis_pos=ex, search=0, offset_flag=0, offset_pos=zeroOff,
                          oacc=100, config=0, velAccParamMode=0, overSpeedStrategy=0, speedPercent=10)
        print(f"\nMoveL j1->d1 rtn: {rtn}")

        rtn = robot.MoveL(desc_pos=d2, tool=1, user=0, joint_pos=j2,
                          vel=100, acc=100, ovl=100, blendR=-1, blendMode=0,
                          exaxis_pos=ex, search=0, offset_flag=0, offset_pos=zeroOff,
                          oacc=100, config=0, velAccParamMode=0, overSpeedStrategy=0, speedPercent=10)
        print(f"\nMoveL j2->d2 rtn: {rtn}")

    # Close connection (never reached due to infinite loop)
    robot.CloseRPC()


if __name__ == "__main__":
    main()
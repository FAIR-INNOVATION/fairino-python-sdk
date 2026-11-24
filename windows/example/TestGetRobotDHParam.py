from fairino import Robot
import time


def main():
    # Establish connection with the robot controller
    robot = Robot.RPC('192.168.58.2')
    time.sleep(0.5)  # Wait for connection and data reception

    print("========== Get Robot DH Param Test ==========")

    # Get DH compensation mode
    rtn, compenMode = robot.GetDHCompenMode()
    print(f"GetDHCompenMode {compenMode}")

    if compenMode == 0:
        # Mode 0: get DH compensation values
        retval, dh = robot.GetDHCompensation()
        print(f"retval is: {retval}")
        print(f"dh is: {dh[0]} {dh[1]} {dh[2]} {dh[3]} {dh[4]} {dh[5]}")

    elif compenMode == 1:
        # Mode 1: get all DH parameters
        rtn, d, a, alpha, beta, theta = robot.GetDHAllParam()
        print(f"GetDHAllParam d   {d[0]} {d[1]} {d[2]} {d[3]} {d[4]} {d[5]}")
        print(f"GetDHAllParam a   {a[0]} {a[1]} {a[2]} {a[3]} {a[4]} {a[5]}")
        print(f"GetDHAllParam alpha   {alpha[0]} {alpha[1]} {alpha[2]} {alpha[3]} {alpha[4]} {alpha[5]}")
        print(f"GetDHAllParam beta   {beta[0]} {beta[1]} {beta[2]} {beta[3]} {beta[4]} {beta[5]}")
        print(f"GetDHAllParam theta   {theta[0]} {theta[1]} {theta[2]} {theta[3]} {theta[4]} {theta[5]}")

    time.sleep(1)

    # Close connection
    robot.CloseRPC()
    time.sleep(1)


if __name__ == "__main__":
    main()
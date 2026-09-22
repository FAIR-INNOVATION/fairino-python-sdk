from fairino import Robot
import time


def main():
    # Establish connection with the robot controller
    robot = Robot.RPC('192.168.58.2')
    time.sleep(0.5)  # Wait for connection and data reception

    # Get current TCP pose
    ret, curTcp = robot.GetActualTCPPose(1)
    if ret != 0:
        print(f"GetActualTCPPose failed, ret:{ret}")
        robot.CloseRPC()
        return

    print(f"Current TCP pose: x={curTcp[0]:.3f}, y={curTcp[1]:.3f}, z={curTcp[2]:.3f}, "
          f"a={curTcp[3]:.3f}, b={curTcp[4]:.3f}, c={curTcp[5]:.3f}")

    # Exaxis position, all zeros here
    exPos = [0.0, 0.0, 0.0, 0.0]

    # Call TCFToAllJoint to get 8 inverse solutions, tool=0, workpiece=0
    rtn, allJoints = robot.TCFToAllJoint(curTcp, 0, 0, exPos)
    if rtn != 0:
        print(f"TCFToAllJoint failed, rtn:{rtn}")
        robot.CloseRPC()
        return

    # Verify each solution with forward kinematics
    for i in range(8):
        joint = allJoints[i]

        print(f"JOINT{i + 1}: j1={joint[0]:.3f}, j2={joint[1]:.3f}, j3={joint[2]:.3f}, "
              f"j4={joint[3]:.3f}, j5={joint[4]:.3f}, j6={joint[5]:.3f}")

        # Forward kinematics to get cartesian pose of the i-th solution
        rtn, pos = robot.GetForwardKin(joint)
        if rtn != 0:
            # Skip this solution if forward kinematics fails
            continue

        print(f"POS{i + 1}: x={pos[0]:.3f}, y={pos[1]:.3f}, z={pos[2]:.3f}, "
              f"a={pos[3]:.3f}, b={pos[4]:.3f}, c={pos[5]:.3f}")

    # Close connection
    robot.CloseRPC()


if __name__ == "__main__":
    main()
from fairino import Robot
import time


def main():
    # Establish connection with the robot controller
    robot = Robot.RPC('192.168.58.2')
    time.sleep(0.5)  # Wait for connection and data reception

    print("========== Weave with Swing Test ==========")

    # Joint positions
    j1 = [-72.561, -38.933, 92.047, -143.101, -90.000, 17.439]
    j2 = [-100.211, -44.358, 105.565, -151.192, -90.007, -10.211]

    # Cartesian poses
    desc_pos1 = [-300.008, 599.999, -199.999, -0.013, -0.004, 0.000]
    desc_pos2 = [-0.004, 599.977, -200.038, -0.017, -0.004, 0.000]

    offset_pos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    epos = [0.0, 0.0, 0.0, 0.0]

    tool = 1
    user = 0
    vel = 100.0
    acc = 100.0
    ovl = 100.0
    oacc = 100.0
    blendR = -1.0
    flag = 0
    search = 0
    blendMode = 0
    velAccMode = 0

    # Set speed to 5%
    robot.SetSpeed(5)

    # Set weave parameters
    rtn = robot.WeaveSetPara(0, 10, 1.000000, 0, 2.000000, 0.000000, 0.000000, 0, 50, 80, 0, 1, 0.000000, 0.000000, 20, 15)
    print(f"WeaveSetPara rtn is: {rtn}")

    # Move to start point
    rtn = robot.MoveJ(joint_pos=j1, desc_pos=desc_pos1, tool=tool, user=user,
                      vel=vel, acc=acc, ovl=ovl, exaxis_pos=epos,
                      blendT=-1.0, offset_flag=flag, offset_pos=offset_pos)
    print(f"movej errcode:{rtn}")

    # Start weaving
    robot.WeaveStart(0)

    # MoveL with weaving
    rtn = robot.MoveL(desc_pos=desc_pos2, tool=tool, user=user, joint_pos=j2,
                      vel=vel, acc=acc, ovl=ovl, blendR=blendR, blendMode=blendMode,
                      exaxis_pos=epos, search=search, offset_flag=flag, offset_pos=offset_pos,
                      oacc=oacc, config=-1, velAccParamMode=velAccMode)
    print(f"movel errcode:{rtn}")

    # End weaving
    robot.WeaveEnd(0)

    time.sleep(1)

    # Close connection
    robot.CloseRPC()
    time.sleep(1)


if __name__ == "__main__":
    main()
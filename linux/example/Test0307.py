from fairino import Robot
import time
# A connection is established with the robot controller. A successful connection returns a robot object
robot = Robot.RPC('192.168.58.2')

def test0307(self):
    print("start")

    # while True:
    #     start = time.time()
    #     rtn = robot.LuaUpload("D://zUP/27.lua")
    #     end = time.time()
    #     print("file upload time is : ",(end - start),"    rtn is ",rtn)
    #     time.sleep(0.1)

    while True:
        p1Desc = [327.604, -104.449, 469.385, -170.284, 2.548, -75.248]
        p1Joint = [-1.953, -64.856, -112.831, -83.705, 95.191, -16.097]

        p2Desc = [305.959, 260.947, 408.152, 177.966, 0.257, -18.624]
        p2Joint = [55.276, -76.083, -111.563, -84.379, 89.683, -16.099]

        exaxisPos = [0.0, 0.0, 0.0, 0.0]
        offdese = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        rtn = robot.MoveL(desc_pos=p1Desc, tool=0, user=0)
        if rtn != 0:
            print("moveL rtn is ",rtn)
            break
        rtn = robot.MoveL(desc_pos=p2Desc, tool=0, user=0)
        if rtn != 0:
            print("moveL rtn is ", rtn)
            break


    # while True:
    #     p1Desc = [-327.459, -378.978, 458.942, 172.127, 38.377, 115.097]
    #     p1Joint = [30.004, -91.868, 96.111, -88.079, -51.359, 0.000]
    #
    #     p2Desc = [29.900, -499.960, 458.942, 172.127, 38.377, 159.348]
    #     p2Joint = [74.255, -91.868, 96.111, -88.079, -51.359, 0.000]
    #
    #     exaxisPos = [0.0, 0.0, 0.0, 0.0]
    #     offdese = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    #     rtn = robot.MoveL(desc_pos=p1Desc, tool=0, user=0)
    #     if rtn != 0:
    #         print("moveL rtn is ", rtn)
    #         break
    #     rtn = robot.MoveL(desc_pos=p2Desc, tool=0, user=0)
    #     if rtn != 0:
    #         print("moveL rtn is ", rtn)
    #         break

    i=0
    while True:
        if i==10:
            robot.ResetAllError()
        i = i+1
        print("main code is ",robot.robot_state_pkg.main_code,"  sub code is  ",robot.robot_state_pkg.sub_code)
        time.sleep(0.1)

    robot.CloseRPC()
    return 0

test0307(robot)
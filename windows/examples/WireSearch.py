from fairino import Robot
import time
# Establishes a connection with the robot controller and returns a robot object if the connection is successful

robot = Robot.RPC('192.168.58.2')

refPos = 1 #  1-datum point 2-reference point
searchVel = 10 #positioning speed %
searchDis = 100 #positioning distanc mm
autoBackFlag = 0 #automatic return flag，0-not automatic；1-automatic 
autoBackVel = 10 #automatic return speed %
autoBackDis = 100 #automatic return distance mm
offectFlag = 0  #1-With offset location；2-Teaching point location

descStart =[203.061, 56.768, 62.719, -177.249, 1.456, -83.597]
jointStart = [-127.012, -112.931, -94.078, -62.014, 87.186, 91.326]
descEnd = [122.471, 55.718, 62.209, -177.207, 1.375, -76.310]
jointEnd = [-119.728, -113.017, -94.027, -62.061, 87.199, 91.326]

robot.MoveL(descStart,1,1,joint_pos= jointStart,vel=100)
robot.MoveL(descEnd,1,1,joint_pos= jointEnd,vel=100)

descREF0A = [147.139, -21.436, 60.717, -179.633, -3.051, -83.170]
jointREF0A = [-121.731, -106.193, -102.561, -64.734, 89.972, 96.171]

descREF0B = [139.247, 43.721, 65.361, -179.634, -3.043, -83.170]
jointREF0B = [-122.364, -113.991, -90.860, -68.630, 89.933, 95.540]

descREF1A = [289.747, 77.395, 58.390, -179.074, -2.901, -89.790]
jointREF1A =[-135.719, -119.588, -83.454, -70.245, 88.921, 88.819]

descREF1B = [259.310, 79.998, 64.774, -179.073, -2.900, -89.790]
jointREF1B =[-133.133, -119.029, -83.326, -70.976, 89.069, 91.401]

error = robot.WireSearchStart(0, 10, 100, 0, 10, 100, 0)
print("WireSearchStart return:",error)

robot.MoveL(descREF0A,1,1, joint_pos = jointREF0A, vel=100)
print("MoveL(descREF0A return:",error)
robot.MoveL(descREF0B,1,1, joint_pos = jointREF0B, vel=10,search=1)
print("MoveL(descREF0B return:",error)

error =robot.WireSearchWait("REF0")
print("WireSearchWait return:",error)

error = robot.WireSearchEnd(0, 10, 100, 0, 10, 100, 0)
print("WireSearchEnd return:",error)


error = robot.WireSearchStart(1,10,100,0,10,100,0)
print("WireSearchStart return:",error)

robot.MoveL(descREF1A,1,1, joint_pos = jointREF1A, vel=100)
robot.MoveL(descREF1B,1,1, joint_pos = jointREF1B, vel=10,search=1)

error =robot.WireSearchWait("REF1")
print("WireSearchWait return:",error)

error = robot.WireSearchEnd(1,10,100,0,10,100,0)
print("WireSearchEnd return:",error)



error = robot.WireSearchStart(1,10,100,0,10,100,0)
print("WireSearchStart return:",error)

robot.MoveL(descREF0A,1,1, joint_pos = jointREF0A, vel=100)
robot.MoveL(descREF0B,1,1, joint_pos = jointREF0B, vel=10,search=1)

error =robot.WireSearchWait("RES0")
print("WireSearchWait return:",error)

error = robot.WireSearchEnd(1,10,100,0,10,100,0)
print("WireSearchEnd return:",error)


error = robot.WireSearchStart(1,10,100,0,10,100,0)
print("WireSearchStart return:",error)

robot.MoveL(descREF1A,1,1, joint_pos = jointREF1A, vel=100)
robot.MoveL(descREF1B,1,1, joint_pos = jointREF1B, vel=10,search=1)

error =robot.WireSearchWait("RES1")
print("WireSearchWait return:",error)

error = robot.WireSearchEnd(1,10,100,0,10,100,0)
print("WireSearchEnd return:",error)

varNameRef = ["REF0", "REF1", "#", "#", "#", "#"]
varNameRes = ["RES0", "RES1", "#", "#", "#", "#"]
error = robot.GetWireSearchOffset(0, 0, varNameRef, varNameRes)
print("GetWireSearchOffect return:",error)
if error[0]==0:
    ref = error[1]
    offdesc =error[2]

    error = robot.PointsOffsetEnable(ref,offdesc)
    print("PointsOffsetEnable return:",error)

    error = robot.MoveL(descStart, 1, 1, joint_pos=jointStart, vel=100)
    print("MoveL return:",error)
    robot.MoveL(descEnd, 1, 1, joint_pos=jointEnd, vel=10)
    print("MoveL return:",error)
    error = robot.PointsOffsetDisable()
    print("PointsOffsetDisable return:",error)



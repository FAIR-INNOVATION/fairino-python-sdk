from fairino import Robot
import time
#  A connection is established with the robot controller. A successful connection returns a robot object
robot = Robot.RPC('192.168.58.2')

desc_pos1 = [36.794,-475.119, 65.379, -176.938, 2.535, -179.829]
desc_pos2 = [136.794,-475.119, 65.379, -176.938, 2.535, -179.829]
desc_pos3 = [236.794,-475.119, 65.379, -176.938, 2.535, -179.829]
joint_pos4 = [-83.24, -96.476, 93.688, -114.079, -62, -100]
joint_pos5 = [-43.24, -70.476, 93.688, -114.079, -62, -80]
joint_pos6 = [-83.24, -96.416, 43.188, -74.079, -80, -10]

desc_pos7 = [236.794,-475.119, 65.379, -176.938, 2.535, -179.829]
desc_pos8 = [236.794,-575.119, 165.379, -176.938, 2.535, -179.829]
desc_pos9 = [236.794,-475.119, 265.379, -176.938, 2.535, -179.829]
tool = 0 # Tool number
user = 0 # Workpiece number


ret = robot.MoveL(desc_pos1, tool, user)    # Rectilinear motion in Cartesian space
print("Rectilinear motion in Cartesian space, Point1:errcode ", ret)

robot.MoveL(desc_pos2, tool, user, vel=30, acc=100)
print("Rectilinear motion in Cartesian space, Point2:errcode ", ret)

robot.MoveL(desc_pos3, tool, user, offset_flag=1, offset_pos=[10,10,10,0,0,0])
print("Rectilinear motion in Cartesian space, Point3:errcode ", ret)

ret = robot.MoveJ(joint_pos4, tool, user, vel=30)   # Joint space motionPTP, the actual test is based on field data ,Tool number and Workpiece number
print("Joint space motion Point 4:errcode", ret)

ret = robot.MoveJ(joint_pos5, tool, user)
print("Joint space motion Point 5:errcode", ret)

robot.MoveJ(joint_pos6, tool, user, offset_flag=1, offset_pos=[10,10,10,0,0,0])
print("Joint space motion Point 6:errcode", ret)

robot.MoveCart(desc_pos7, tool, user)
print("Point-to-point motion in Cartesian space: ", ret)

robot.MoveCart(desc_pos8, tool, user, vel=30)
print("Point-to-point motion in Cartesian space: ", ret)

robot.MoveCart(desc_pos9, tool, user,)
print("Point-to-point motion in Cartesian space: ", ret)
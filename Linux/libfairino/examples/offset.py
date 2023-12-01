from fairino import Robot
# A connection is established with the robot controller. A successful connection returns a robot object
robot = Robot.RPC('192.168.58.2')

desc_pos1 = [-187.519, 319.248, 397, -157.278, -31.188, 107.199]
desc_pos2 = [-187.519, 310.248, 297, -157.278, -31.188, 107.199]
desc_pos3 = [-127.519, 256.248, 312, -147.278, -51.588, 107.199]
joint_pos1 = [-83.24, -96.476, 93.688, -114.079, -62, -100]
desc_pos4 = [-140.519, 219.248, 300, -137.278, -11.188, 127.199]
desc_pos5 = [-187.519, 319.248, 397, -157.278, -31.188, 107.199]
desc_pos6 = [-207.519, 229.248, 347, -157.278, -31.188, 107.199]

tool = 0 #Tool number
user = 0 #Workpiece number
flag = 1
offset_pos = [10,20,30,0,0,0]

ret = robot.MoveL(desc_pos1, tool, user, joint_pos=joint_pos1)   # Linear motion in Cartesian space
print("Linear motion in Cartesian space:errcode", ret)
ret = robot.StopMotion()  # Stop Motion
print("Stop Motion: errcode ", ret)
robot.MoveL(desc_pos2, tool, user, vel=40, acc=100)
print("Linear motion in Cartesian space:errcode", ret)

ret = robot.PointsOffsetEnable(flag,offset_pos)
print("Starting point overall offset:errcode", ret)
robot.MoveL(desc_pos3, tool, user, offset_flag=1, offset_pos=[10,10,10,0,0,0])
print("Linear motion in Cartesian space:errcode ", ret)
robot.MoveL(desc_pos4, tool, user, vel=30, acc=100)
print("Linear motion in Cartesian space:errcode ", ret)
robot.MoveL(desc_pos5, tool, user)
print("Linear motion in Cartesian space:errcode ", ret)
ret = robot.PointsOffsetDisable()
print("The overall offset of the point ends:errcode ", ret)

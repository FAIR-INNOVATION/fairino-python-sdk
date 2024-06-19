from fairino import Robot
import time
# # A connection is established with the robot controller. A successful connection returns a robot object
robot = Robot.RPC('192.168.58.2')
robot.Mode(0)
time.sleep(1)
e_pos =[-20,0,0,0]
joint_pos0 = [114.089,-85.740, 119.106,-129.884,-91.655, 79.642]
desc_pos0= [-87.920,-178.539,-64.513,-175.471,7.664,139.650]
#The UDP expansion axis moves synchronously with the robot joint movement
error = robot.ExtAxisSyncMoveJ(joint_pos0,desc_pos0,1,1,e_pos)
print("ExtAxisSyncMoveJ",error)
time.sleep(3)

#The UDP extension axis moves synchronously with the robot's linear motion
error = robot.ExtAxisSyncMoveL(joint_pos0,desc_pos0,1,1,e_pos)
print("ExtAxisSyncMoveL",error)
time.sleep(3)

desc_pos_mid =[-131.2748107910156, -60.21242523193359, -22.55266761779785, 175.9907989501953, 5.92541742324829, 145.5211791992187]
desc_pos_end =[-91.3530502319336, -174.5040588378906, -64.93866729736328, 177.1370544433593, 15.96347618103027, 136.1746368408203]
joint_pos_mid = [120.9549040841584, -109.8869943146658, 134.1448068146658, -126.2150709699876, -88.6738087871287, 79.6419593131188]
joint_pos_end =[110.1896078279703, -89.01601659189356, 125.5532806698638, -139.7967831451114, -82.93198387221534, 79.6452225788985]
#The UDP extension axis moves synchronously with the robot arc motion
time.sleep(3)
error = robot.ExtAxisSyncMoveC(joint_pos_mid,desc_pos_mid,1,1,[-10,0,0,0],joint_pos_end,desc_pos_end,1,1,[-20,0,0,0])
print("ExtAxisSyncMoveC",error)
time.sleep(3)

error,joint_pos = robot.GetActualJointPosDegree()
print("GetActualJointPosDegree",error,joint_pos)
e_pos =[-10,0,0,0]
joint_pos[0] = joint_pos[0]+30
#UDP extension axis asynchronous movement
error = robot.ExtAxisMove(e_pos,30)
print("ExtAxisMove",error)
print("joint_pos",joint_pos)
error = robot.MoveJ(joint_pos,0,0,exaxis_pos=e_pos)
print("MoveJ",error)



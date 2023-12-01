from fairino import Robot
# A connection is established with the robot controller. A successful connection returns a robot object
robot = Robot.RPC('192.168.58.2')
desc_pos1 = [236.794,-475.119, 65.379, -176.938, 2.535, -179.829]
desc_posc1 = [266.794,-455.119, 65.379, -176.938, 2.535, -179.829] #MoveC Path point
desc_posc2 = [286.794,-475.119, 65.379, -176.938, 2.535, -179.829]  #MoveC Target point
desc_pos2 = [236.794,-475.119, 65.379, -176.938, 2.535, -179.829]
desc_posc3 = [256.794,-435.119, 65.379, -176.938, 2.535, -179.829]   #Circle Path point
desc_posc4 = [286.794,-475.119, 65.379, -176.938, 2.535, -179.829]  #Circle Target point
desc_pos3 = [236.794,-475.119, 65.379, -176.938, 2.535, -179.829]
desc_pos_spiral= [236.794,-475.119, -65.379, -176.938, 2.535, -179.829]#Spiral Target point

#Spiral param[circle_num, circle_angle, rad_init, rad_add, rotaxis_add, rot_direction]
param = [5.0,10,30,10,5,0]
tool = 0 # Tool number
user = 0 # Workpiece number
ret = robot.MoveL(desc_pos1, tool, user, vel=30, acc=100)
print("Linear motion in Cartesian space: errorcode", ret)

ret = robot.MoveC(desc_posc1, tool, user, desc_posc2,tool, user)  #Circular arc motion in Cartesian space
print("Circular arc motion in Cartesian space:errcode", ret)

robot.MoveL(desc_pos2, tool, user, vel=40, acc=100)#Circular motion in Cartesian space
print("Linear motion in Cartesian space: errorcode", ret)

ret = robot.Circle(desc_posc3, tool, user, desc_posc4, tool, user, vel_t=40, offset_flag=1, offset_pos=[5,10,15,0,0,1])  #笛卡尔空间圆弧运动
print("Circular motion in Cartesian space:errcode", ret)

ret = robot.NewSpiral(desc_pos_spiral, tool, user, param,vel=40 )  #Spiral motion in Cartesian space
print("Spiral motion in Cartesian space:errcode", ret)
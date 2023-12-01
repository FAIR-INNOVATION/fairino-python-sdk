from fairino import Robot
# A connection is established with the robot controller. A successful connection returns a robot object
robot = Robot.RPC('192.168.58.2')

#Constant force control
status = 1  #Constant force control open flag, 0-off, 1-on
sensor_num = 1 #Force sensor number
is_select = [0,0,1,0,0,0]  #Six degrees of freedom choice[fx,fy,fz,mx,my,mz],0-ineffective, 1-effective
force_torque = [0.0,0.0,-10.0,0.0,0.0,0.0]  #Collision detection force and torque, detection range（force_torque-min_threshold,force_torque+max_threshold）
gain = [0.0005,0.0,0.0,0.0,0.0,0.0]  #Maximum threshold
adj_sign = 0  #Adaptive start stop status, 0-off, 1-on
ILC_sign = 0  #ILC control start stop status, 0-stop, 1-training, 2-practical operation
max_dis = 100.0  #Maximum adjustment distance
max_ang = 0.0  #Maximum adjustment angle
J1=[70.395, -46.976, 90.712, -133.442, -87.076, -27.138]
P2=[-123.978, -674.129, 44.308, -178.921, 2.734, -172.449]
P3=[123.978, -674.129, 42.308, -178.921, 2.734, -172.449]
error = robot.MoveJ(J1,1,0)
print("Joint space motion PTP ",error)
error = robot.MoveL(P2,1,0)
print("Linear motion in Cartesian space ",error)
error = robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign, ILC_sign,max_dis,max_ang)
print("Turn on constant force control ",error)
error = robot.MoveL(P3,1,0)
print("Linear motion in Cartesian space ",error)
status = 0
error = robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign, ILC_sign,max_dis,max_ang)
print("Turn off constant force control ",error)

from fairino import Robot
# A connection is established with the robot controller. A successful connection returns a robot object
robot = Robot.RPC('192.168.58.2')

P = [36.794,-675.119, 65.379, -176.938, 2.535, -179.829]
#Constant force parameter
status = 1  #Constant force control open flag, 0-off, 1-on
sensor_num = 1 #Force sensor number
is_select = [0,0,1,0,0,0]  #Six degrees of freedom choice[fx,fy,fz,mx,my,mz],0-ineffective, 1-effective
force_torque = [0.0,0.0,-10.0,0.0,0.0,0.0]  #Collision detection force and torque, detection range（force_torque-min_threshold,force_torque+max_threshold）
gain = [0.0001,0.0,0.0,0.0,0.0,0.0]  #Maximum threshold
adj_sign = 0  #Adaptive start stop status, 0-off, 1-on
ILC_sign = 0  #ILC control start stop status, 0-stop, 1-training, 2-practical operation
max_dis = 100.0  #Maximum adjustment distance
max_ang = 5.0  #Maximum adjustment angle
#Rotational insertion parameter
rcs = 0  #Reference frame, 0-Tool frame, 1-Base frame
angVelRot = 2.0  #Rotational angular velocity,unit[°/s]
forceInsertion = 1.0 #Force or moment threshold（0~100）,unit[N or Nm]
angleMax= 45 #Maximum rotation Angle,unit[°]
orn = 1 #Direction of force,1-fz,2-mz
angAccmax = 0.0 #Maximum rotational acceleration, unit[°/s^2],not used temporarily
rotorn = 1 #Rotation direction, 1-clockwise, 2-counterclockwise

error = robot.MoveL(P,1,0) # Linear motion in Cartesian space
print("Linear motion in Cartesian space ",error)
error = robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign, ILC_sign,max_dis,max_ang)
print("Turn on constant force control ",error)
error = robot.FT_RotInsertion(rcs,1,orn)
print("Rotate Insert ",error)
status = 0
error = robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign, ILC_sign,max_dis,max_ang)
print("Turn off constant force control ",error)


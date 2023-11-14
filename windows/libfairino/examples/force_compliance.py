from fairino import Robot
import time
# A connection is established with the robot controller. A successful connection returns a robot object
robot = Robot.RPC('192.168.58.2')

J1=[75.005,-46.434,90.687,-133.708,-90.315,-27.139]
P2=[-77.24,-679.599,38.328,179.373,-0.028,-167.849]
P3=[77.24,-679.599,38.328,179.373,-0.028,-167.849]
#Constant force parameter
status = 1  #Constant force control open flag, 0-off, 1-on
sensor_num = 1 #Force sensor number
is_select = [1,1,1,0,0,0]  #Six degrees of freedom choice[fx,fy,fz,mx,my,mz],0-ineffective, 1-effective
force_torque = [-10.0,-10.0,-10.0,0.0,0.0,0.0] #Collision detection force and torque, detection range（force_torque-min_threshold,force_torque+max_threshold）
gain = [0.0005,0.0,0.0,0.0,0.0,0.0]  #
adj_sign = 0  #Adaptive start stop status, 0-off, 1-on
ILC_sign = 0  #ILC control start stop status, 0-stop, 1-training, 2-practical operation
max_dis = 1000.0  #Maximum adjustment distanc
max_ang = 0.0  #Maximum adjustment angle
error = robot.MoveJ(J1,1,0)
#Compliance control
error = robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign, ILC_sign,max_dis,max_ang)
print("Turn on constant force control ",error)
p = 0.00005  # Coefficient of position adjustment or compliance
force = 30.0 # Compliant opening force threshold,unit[N]
error = robot.FT_ComplianceStart(p,force)
print("Turn on flexibility control ",error)
error = robot.MoveL(P2,1,0,vel =10)   # Rectilinear motion in Cartesian space
print("Rectilinear motion in Cartesian space ", error)
error = robot.MoveL(P3,1,0,vel =10)
print("Rectilinear motion in Cartesian space ", error)
time.sleep(1)
error = robot.FT_ComplianceStop()
print("Turn off flexibility control ",error)
status = 0
error = robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign, ILC_sign,max_dis,max_ang)
print("Turn off constant force control ",error)

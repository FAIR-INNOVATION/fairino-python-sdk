from fairino import Robot
import time
# A connection is established with the robot controller. A successful connection returns a robot object
robot = Robot.RPC('192.168.58.2')

#Constant force parameter
status = 1  #Constant force control open flag, 0-off, 1-on
sensor_num = 1 #Force sensor number
is_select = [1,0,0,0,0,0]  #Six degrees of freedom choice[fx,fy,fz,mx,my,mz],0-ineffective, 1-effective
force_torque = [-2.0,0.0,0.0,0.0,0.0,0.0]  #Collision detection force and torque, detection range（force_torque-min_threshold,force_torque+max_threshold）
gain = [0.0002,0.0,0.0,0.0,0.0,0.0]  #Maximum threshold
adj_sign = 0  #Adaptive start stop status, 0-off, 1-on
ILC_sign = 0  #ILC control start stop status, 0-stop, 1-training, 2-practical operation
max_dis = 100.0  #Maximum adjustment distance
max_ang = 5.0  #Maximum adjustment angle
#Surface positioning parameter
rcs = 0 #Reference frame, 0-Tool frame, 1-Base frame
direction = 1 #Direction of movement,1-positive direction, 2-negative direction
axis = 1 #Axis of movement,1-X,2-Y,3-Z
lin_v = 3.0  #Exploring straight-line velocity,unit[mm/s]
lin_a = 0.0  #Exploration linear acceleration,unit[mm/s^2]
disMax = 50.0 #Maximum exploration distance,unit[mm]
force_goal = 2.0 #Action termination force threshold,unit[N]

P1=[-77.24,-679.599,58.328,179.373,-0.028,-167.849]
robot.MoveCart(P1,1,0)       #Point to point motion in joint space
#Look for the center in the x direction
#The first surface
error = robot.FT_CalCenterStart()
error = robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign,ILC_sign,max_dis,max_ang)
error = robot.FT_FindSurface(rcs,direction,axis,disMax,force_goal)
status = 0
error = robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign,ILC_sign,max_dis,max_ang)
time.sleep(2)
error = robot.MoveCart(P1,1,0)       #Point to point motion in joint space
time.sleep(5)
#The second surface
error = robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign,ILC_sign,max_dis,max_ang)
direction = 2 #Direction of movement,1-positive direction, 2-negative direction
error = robot.FT_FindSurface(rcs,direction,axis,disMax,force_goal)
status = 0
error = robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign,ILC_sign,max_dis,max_ang)
#Calculate the x-direction center position
error,xcenter = robot.FT_CalCenterEnd()
print("xcenter",xcenter)
error = robot.MoveCart(xcenter,1,0)
time.sleep(1)

#Look for the center in the y direction
#The first surface
error =robot.FT_CalCenterStart()
error =robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign,ILC_sign,max_dis,max_ang)
direction = 1 #Direction of movement,1-positive direction, 2-negative direction
axis = 2 #Axis of movement,1-X,2-Y,3-Z
disMax = 150.0 #Maximum exploration distance,unit[mm]
lin_v = 6.0  #Exploring straight-line velocity,unit[mm/s]
error =robot.FT_FindSurface(rcs,direction,axis,disMax,force_goal)
status = 0
error =robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign,ILC_sign,max_dis,max_ang)
error =robot.MoveCart(P1,1,0)       #Point to point motion in joint space
robot.WaitMs(1000)
#The second surface
error =robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign,ILC_sign,max_dis,max_ang)
direction = 2 #Direction of movement,1-positive direction, 2-negative direction
error =robot.FT_FindSurface(rcs,direction,axis,disMax,force_goal)
status = 0
error =robot.FT_Control(status,sensor_num,is_select,force_torque,gain,adj_sign,ILC_sign,max_dis,max_ang)
#Calculate the y center position
error,ycenter=robot.FT_CalCenterEnd()
print("y center position",ycenter)
error =robot.MoveCart(ycenter,1,0)
from fairino import Robot
# A connection is established with the robot controller. A successful connection returns a robot object
robot = Robot.RPC('192.168.58.2')
actFlag = 1   #Enable flag, 0-Disable collision guard, 1-Enable collision guard
sensor_num = 1  #Force sensor number
is_select = [1,1,1,1,1,1]  #Whether the six degrees of freedom detect the collision[fx,fy,fz,mx,my,mz],0-Ineffective, 1-Effective
force_torque = [0.0,0.0,0.0,0.0,0.0,0.0]  #Collision detection force/moment,detection range（force_torque-min_threshold,force_torque+max_threshold）
max_threshold = [10.0,10.0,10.0,10.0,10.0,10.0]  #Maximum threshold
min_threshold = [5.0,5.0,5.0,5.0,5.0,5.0]   #Minimum Threshold
P1=[-160.619,-586.138,384.988,-170.166,-44.782,169.295]
P2=[-87.615,-606.209,556.119,-102.495,10.118,178.985]
P3=[41.479,-557.243,484.407,-125.174,46.995,-132.165]
error = robot.FT_Guard(actFlag, sensor_num, is_select, force_torque, max_threshold, min_threshold)    # Turn on collision protection
print("Turn on collision protection ",error)
error = robot.MoveL(P1,1,0)         # linear motion in Cartesian space
print("Linear motion in Cartesian space ",error)
error = robot.MoveL(P2,1,0)
print("Linear motion in Cartesian space ",error)
error = robot.MoveL(P3,1,0)
print("Linear motion in Cartesian space ",error)
actFlag = 0
error = robot.FT_Guard(actFlag, sensor_num, is_select, force_torque, max_threshold, min_threshold)    # Turn off collision protection
print("Turn off collision protection ",error)

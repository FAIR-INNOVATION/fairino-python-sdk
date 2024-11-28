from fairino import Robot
import time
# 与机器人控制器建立连接，连接成功返回一个机器人对象

robot = Robot.RPC('192.168.58.2')
mulitilineorigin1_joint=[-24.090,-63.501,84.288,-111.940,-93.426,57.669]
mulitilineorigin1_desc=[-677.559,190.951,-1.205,1.144,-41.482,-82.577]
mulitilineX1_joint=[-25.734,-62.843,83.315,-111.723,-93.392,56.021]
mulitilineX1_desc=[-677.556,211.949,-1.206,1.145,-41.482,-82.577]
mulitilineZ1_joint=[-24.090,-64.449,82.477,-109.183,-93.427,57.668]
mulitilineZ1_desc=[-677.564,190.956,19.817,1.143,-41.481,-82.576]
mulitilinesafe_joint=[-25.734,-63.778,81.502,-108.975,-93.392,56.021]
mulitilinesafe_desc=[-677.561,211.950,19.812,1.144,-41.482,-82.577]


mulitilineorigin2_joint=[-29.743,-75.623,101.241,-116.354,-94.928,55.735]
mulitilineorigin2_desc=[-563.961,215.359,-0.681,2.845,-40.476,-87.443]
mulitilineX2_joint=[-30.182,-75.433,101.005,-116.346,-94.922,55.294]
mulitilineX2_desc=[-563.965,220.355,-0.680,2.845,-40.476,-87.442]
mulitilineZ2_joint=[-29.743,-75.916,100.817,-115.637,-94.928,55.735]
mulitilineZ2_desc=[-563.968,215.362,4.331,2.844,-40.476,-87.442]

time.sleep(10)
error=robot.MoveJ(mulitilinesafe_joint,13,0,vel =10)
print("MoveJ return:",error)

error=robot.MoveL(mulitilineorigin1_desc,13,0,vel =10)
print("MoveL return:",error)

error=robot.MoveJ(mulitilinesafe_joint,13,0,vel =10)
print("MoveJ return:",error)

error=robot.MoveL(mulitilineorigin2_desc,13,0,vel =10)
print("MoveL return:",error)

error=robot.MoveJ(mulitilinesafe_joint,13,0,vel =10)
print("MoveJ return:",error)

error=robot.MoveJ(mulitilineorigin1_joint,13,0,vel =10)
print("MoveJ return:",error)

error=robot.ARCStart(1, 0, 3000)
print("ARCStart return:",error)

error=robot.WeaveStart(0)
print("WeaveStart return:",error)

error = robot.ArcWeldTraceControl(1,0,1,0.06,5,5,50,1,0.06,5,5,55,0,0,4,1,10)
print("ArcWeldTraceControl return:",error)

error=robot.MoveL(mulitilineorigin2_desc,13,0,joint_pos=mulitilineorigin2_joint,vel =1)
print("MoveL return:",error)

error = robot.ArcWeldTraceControl(0,0,1,0.06,5,5,50,1,0.06,5,5,55,0,0,4,1,10)
print("ArcWeldTraceControl return:",error)

error=robot.WeaveEnd(0)
print("WeaveEnd return:",error)

error=robot.ARCEnd(1, 0, 10000)
print("ARCEnd return:",error)


error=robot.MoveJ(mulitilinesafe_joint,13,0,vel =10)
print("MoveJ return:",error)

error,offset = robot.MultilayerOffsetTrsfToBase(mulitilineorigin1_desc,mulitilineX1_desc,mulitilineZ1_desc,10,0,0)
print("ArcWeldTraceControl return:",error,offset)

error=robot.MoveJ(mulitilineorigin1_joint,13,0,vel =10,offset_flag=1,offset_pos=offset)
print("MoveJ return:",error)

error=robot.ARCStart(1, 0, 3000)
print("ARCStart return:",error)

error,offset = robot.MultilayerOffsetTrsfToBase(mulitilineorigin2_desc,mulitilineX2_desc,mulitilineZ2_desc,10,0,0)
print("ArcWeldTraceControl return:",error,offset)

error = robot.ArcWeldTraceReplayStart()
print("ArcWeldTraceReplayStart return:",error)

error=robot.MoveL(mulitilineorigin2_desc,13,0,joint_pos=mulitilineorigin2_joint,vel =2,offset_flag=1,offset_pos=offset)
print("MoveL return:",error)

error = robot.ArcWeldTraceReplayEnd()
print("ArcWeldTraceReplayEnd return:",error)

error=robot.ARCEnd(1, 0, 10000)
print("ARCEnd return:",error)

error=robot.MoveJ(mulitilinesafe_joint,13,0,vel =10)
print("MoveJ return:",error)

error,offset = robot.MultilayerOffsetTrsfToBase(mulitilineorigin1_desc,mulitilineX1_desc,mulitilineZ1_desc,0,10,0)
print("ArcWeldTraceControl return:",error,offset)

error=robot.MoveJ(mulitilineorigin1_joint,13,0,vel =10,offset_flag=1,offset_pos=offset)
print("MoveJ return:",error)

error=robot.ARCStart(1, 0, 3000)
print("ARCStart return:",error)

error,offset = robot.MultilayerOffsetTrsfToBase(mulitilineorigin2_desc,mulitilineX2_desc,mulitilineZ2_desc,0,10,0)
print("ArcWeldTraceControl return:",error,offset)

error = robot.ArcWeldTraceReplayStart()
print("ArcWeldTraceReplayStart return:",error)

error=robot.MoveL(mulitilineorigin2_desc,13,0,joint_pos=mulitilineorigin2_joint,vel =2,offset_flag=1,offset_pos=offset)
print("MoveL return:",error)

error = robot.ArcWeldTraceReplayEnd()
print("ArcWeldTraceReplayEnd return:",error)

error=robot.ARCEnd(1, 0, 3000)
print("ARCEnd return:",error)

error=robot.MoveJ(mulitilinesafe_joint,13,0,vel =10)
print("MoveJ return:",error)




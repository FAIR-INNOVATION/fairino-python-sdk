from fairino import Robot
import time
# A connection is established with the robot controller. A successful connection returns a robot object
robot = Robot.RPC('192.168.58.2')
name = "/fruser/traj/trajHelix_aima_1.txt"
blend = 1
ovl = 50.0
ft =[0.5, 0.5, 0.5, 0.5, 0.5, 0.5]
ret = robot.LoadTrajectoryJ(name,ovl)  #Trajectory preloading
print("Trajectory preloading",ret)
ret,P1 = robot.GetTrajectoryStartPose(name)   # Obtain trajectory start pose - Import track files
print ("Obtain trajectory start pose - Import track files ",ret," start pose ",P1)
ret = robot.MoveL(P1,1,0)       # Move to start pose
print("Move tostart pose ",ret)
ret = robot.GetTrajectoryPointNum()# Obtain trajectory point number - Import track files
print("Obtain trajectory point number - Import track files ",ret)
time.sleep(10)
ret = robot.MoveTrajectoryJ()  # Trajectory reproduction - Import track files
print("Trajectory reproduction - Import track files ",ret)
time.sleep(10)
ret = robot.SetTrajectoryJSpeed(ovl)  # Set trajectory speed - Import track files
print("Set trajectory speed - Import track files",ret)
time.sleep(1)
ret = robot.SetTrajectoryJForceTorque(ft)  # Set trajectory force and torque- Import track files
print("Set trajectory force and torque- Import track files ",ret)
time.sleep(1)
ret = robot.SetTrajectoryJForceFx(0) # Set trajectory force Fx- Import track files
print("Set trajectory force Fx- Import track files ",ret)
time.sleep(1)
ret = robot.SetTrajectoryJForceFy(0) # Set trajectory force Fy- Import track files
print("Set trajectory force Fy- Import track files ",ret)
time.sleep(1)
ret = robot.SetTrajectoryJForceFz(0) # Set trajectory force Fz- Import track files
print("Set trajectory force Fx- Import track files ",ret)
time.sleep(1)
ret = robot.SetTrajectoryJTorqueTx(0) # Set trajectory torqueTy- Import track files
print("Set trajectory torqueTy- Import track files ",ret)
time.sleep(1)
ret = robot.SetTrajectoryJTorqueTy(0) # Set trajectory torqueTy- Import track files
print("Set trajectory torqueTy- Import track files ",ret)
time.sleep(1)
ret = robot.SetTrajectoryJTorqueTz(0) # Set trajectory torqueTz- Import track files
print("Set trajectory torqueTz- Import track files ",ret)
time.sleep(1)

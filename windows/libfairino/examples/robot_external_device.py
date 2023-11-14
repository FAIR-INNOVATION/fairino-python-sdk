from fairino import Robot
import time
# A connection is established with the robot controller. A successful connection returns a robot object
robot = Robot.RPC('192.168.58.2')
desc_pos1=[-333.683,-228.968,404.329,-179.138,-0.781,91.261]
desc_pos2=[-333.683,-100.8,404.329,-179.138,-0.781,91.261]
zlength1 =10
zlength2 =15
zangle1 =10
zangle2 =15
ret = robot.SetGripperConfig(4,0)  # Configure gripper
print("Configure gripper ", ret)
time.sleep(1)
config = robot.GetGripperConfig()     # Obtain gripper configuration
print("Obtain gripper configuration ",config)
error = robot.ActGripper(1,0)  # Reset e gripper
print("Reset gripper",error)
time.sleep(1)
error = robot.ActGripper(1,1)# Activate gripper
print("Activate gripper ",error)
time.sleep(2)
error = robot.MoveGripper(1,100,48,46,30000,0) # Control gripper
print("Control gripper ",error)
time.sleep(3)
error = robot.MoveGripper(1,0,50,0,30000,0) # Control gripper
print("Control gripper ",error)
error = robot.GetGripperMotionDone() # Obtain gripper movement status
print("Obtain gripper movement status ",error)
error = robot.ComputePrePick(desc_pos1, zlength1, zangle1) # Calculate pre- pick points - Vision
print("Calculate pre- pick points - Vision",error)
error = robot.ComputePrePick(desc_pos2, zlength2, zangle2) # Calculate post pick points - Vision
print("Calculate post pick points - Vision ",error)

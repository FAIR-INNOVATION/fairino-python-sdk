from fairino import Robot

# A connection is established with the robot controller. A successful connection returns a robot object
robot = Robot.RPC('192.168.58.2')

# Load identification initialization
error = robot.LoadIdentifyDynFilterInit()
print("LoadIdentifyDynFilterInit:",error)

# Load identification variable initialization
error = robot.LoadIdentifyDynVarInit()
print("LoadIdentifyDynVarInit:",error)

joint_torque= [0,0,0,0,0,0]
joint_pos= [0,0,0,0,0,0]
gain=[0,0.05,0,0,0,0,0,0.02,0,0,0,0]
t =10
error,joint_pos=robot.GetActualJointPosDegree(1)
joint_pos[1] = joint_pos[1]+10
error,joint_torque=robot.GetJointTorques(1)
joint_torque[1] = joint_torque[1]+2

# Load identification main program
error = robot.LoadIdentifyMain(joint_torque, joint_pos, t)
print("LoadIdentifyMain:",error)

#Obtain the load identification result
error = robot.LoadIdentifyGetResult(gain)
print("LoadIdentifyGetResult:",error)


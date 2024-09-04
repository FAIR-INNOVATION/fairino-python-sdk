from fairino import Robot
import time
# Establishes a connection with the robot controller and returns a robot object if the connection is successful

robot = Robot.RPC('192.168.58.2')

desc1 = [238.209, -403.633, 251.291, 177.222, -1.433, 133.675]
joint1= [-48.728, -86.235, -95.288, -90.025, 92.715, 87.595]
desc2 = [238.207, -596.305, 251.294, 177.223, -1.432, 133.675]
joint2= [-60.240, -110.743, -66.784, -94.531, 92.351, 76.078 ]

error = robot.MoveL(desc1,1,0,joint_pos=joint1)
print("MoveL return:",error)

error = robot.WeaveInspectStart(0)
print("WeaveInspectStart return:",error)

error = robot.MoveL(desc2,1,0,joint_pos=joint2)
print("MoveL return:",error)

error = robot.WeaveInspectEnd(0)
print("WeaveInspectEnd return:",error)


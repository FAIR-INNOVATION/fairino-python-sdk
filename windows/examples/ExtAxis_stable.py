from fairino import Robot
import time
# A connection is established with the robot controller. A successful connection returns a robot object
robot = Robot.RPC('192.168.58.2')
robot.Mode(0)
e_pos1 =[-20,0,0,0]
joint_pos1 = [115.0179358756188, -81.47939549814356, 115.508295946782, -131.2280998607673, -99.9834177753713, 79.63260461788364]
desc_pos1= [-86.72003173828125, -215.2589569091797, -65.05786895751953, -167.1544799804687, 6.824628829956054, 140.395767211914]
e_pos2 =[0,0,0,0]
joint_pos2 = [127.0652604811262, -91.30226059715346, 118.376053913985, -123.3664574953589, -89.4406762453589, 79.64478747679455]
desc_pos2= [-27.7623996734619, -117.4148635864258, -45.0506019592285, 167.6852264404297, -5.91967725753784, 151.977828979492]
count =500
while count >0:


    error = robot.ExtAxisSyncMoveL(joint_pos2,desc_pos2,1,1,exaxis_pos=e_pos2)
    print("ExtAxisSyncMoveL",error)
    time.sleep(2)
    error = robot.ExtAxisSyncMoveL(joint_pos1,desc_pos1,1,1,exaxis_pos=e_pos1)
    print("ExtAxisSyncMoveL",error)
    time.sleep(2)
    count=count-1
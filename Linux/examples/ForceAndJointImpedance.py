from fairino import Robot
import time
# Establishes a connection with the robot controller and returns a robot object if the connection is successful

robot = Robot.RPC('192.168.58.2')

status = 1 #control state，0-close；1-open
impedanceFlag = 1 #Impedance open sign，0-close；1-open
lamdeDain = [ 3.0, 2.0, 2.0, 2.0, 2.0, 3.0] # drag gain
KGain = [0.00, 0.00, 0.00, 0.00, 0.00, 0.00] # stiffness gain
BGain = [150, 150, 150, 5.0, 5.0, 1.0] # damping gain
dragMaxTcpVel = 1000 #Maximum linear speed limit at the moving end
dragMaxTcpOriVel = 180 #Maximum angular velocity limit at the end of the drag

error = robot.DragTeachSwitch(1)
print("DragTeachSwitch 1  return:",error)

error = robot.ForceAndJointImpedanceStartStop(status, impedanceFlag, lamdeDain, KGain, BGain,dragMaxTcpVel,dragMaxTcpOriVel)
print("ForceAndJointImpedanceStartStop return:",error)

error = robot.GetForceAndTorqueDragState()
print("GetForceAndTorqueDragState return:",error)



time.sleep(10)

status = 0 #control state，0-close；1-open
impedanceFlag = 0 #Impedance open sign，0-close；1-open
error = robot.ForceAndJointImpedanceStartStop(status, impedanceFlag, lamdeDain, KGain, BGain,dragMaxTcpVel,dragMaxTcpOriVel)
print("ForceAndJointImpedanceStartStop return:",error)

error = robot.GetForceAndTorqueDragState()
print("GetForceAndTorqueDragState return:",error)

error = robot.DragTeachSwitch(0)
print("DragTeachSwitch 0  return:",error)
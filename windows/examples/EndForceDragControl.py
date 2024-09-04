from fairino import Robot
import time
# Establishes a connection with the robot controller and returns a robot object if the connection is successful

robot = Robot.RPC('192.168.58.2')

status = 1 #control state, 0-close; 1-open
asaptiveFlag = 1 #Adaptive opening sign, 0-close; 1-open
interfereDragFlag = 1 #Interference zone towing sign, 0-close; 1-open
M = [15, 15, 15, 0.5, 0.5, 0.1] #inertia factor
B = [150, 150, 150, 5, 5, 1] #damping factor
K = [0, 0, 0, 0, 0, 0] #coefficient ofrigidity
F = [5, 5, 5, 1, 1, 1] #Drag the six-dimensional force threshold
Fmax = 50 #Maximum towing power limit
Vmax = 1810 #Maximum joint speed limit

error = robot.EndForceDragControl(status, asaptiveFlag, interfereDragFlag, M, B, K, F, Fmax, Vmax)
print("EndForceDragControl return:",error)

time.sleep(10)
status=0
error = robot.EndForceDragControl(status, asaptiveFlag, interfereDragFlag, M, B, K, F, Fmax, Vmax)
print("EndForceDragControl return:",error)
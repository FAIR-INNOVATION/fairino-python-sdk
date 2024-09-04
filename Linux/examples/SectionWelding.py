from fairino import Robot
import time
# Establishes a connection with the robot controller and returns a robot object if the connection is successful

robot = Robot.RPC('192.168.58.2')
startdescPose =[-91.037, -505.079, 85.895, 164.953, -13.906, -131.826]
startjointPos = [-77.530, -123.147, -60.904, -90.546, 70.124, -33.053]
enddescPose = [261.458, -460.453, 79.089, -171.505, 16.632, -136.465]
endjointPos = [-58.637, -126.187, -60.936, -77.769, 107.931, -10.122]

error = robot.MoveJ(startjointPos,1,0)
print("MoveJ return:",error)

error = robot.SegmentWeldStart(startdescPose,enddescPose,startjointPos,endjointPos,
                           20,20,0,0,
                           5000, 0, 0, 1, 0, vel=100)

print("SegmentWeldStart return:",error)
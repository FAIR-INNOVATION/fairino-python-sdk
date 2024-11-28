from fairino import Robot

robot = Robot.RPC('192.168.58.2')
n=0
while(n<10):
    error, joint = robot.GetActualJointPosDegree()
    print("GetActualJointPosDegree return:", error,joint)

    joint[0]=joint[0]+10
    robot.MoveJ(joint,0,0)
    print("MoveJ return:", error)

    error = robot.GetJointDriverTorque()
    print("GetJointDriverTorque return:", error)

    error = robot.GetJointDriverTemperature()
    print("GetJointDriverTemperature return:", error)

    joint[0]=joint[0]-10
    robot.MoveJ(joint,0,0)
    print("MoveJ return:", error)
    n=n+1
from fairino import Robot

robot = Robot.RPC('192.168.58.2')

error = robot.SetRobotRealtimeStateSamplePeriod(200)
print("SetRobotRealtimeStateSamplePeriod return:", error)

error = robot.GetRobotRealtimeStateSamplePeriod()
print("GetRobotRealtimeStateSamplePeriod return:", error)

n=0
while(n<100):
    error, joint = robot.GetActualJointPosDegree()
    print("GetActualJointPosDegree return:", error,joint)

    joint[1]=joint[1]+10
    robot.MoveJ(joint,0,0)

    error = robot.GetJointDriverTorque()
    print("GetJointDriverTorque return:", error)

    error = robot.GetJointDriverTemperature()
    print("GetJointDriverTemperature return:", error)

    joint[1]=joint[1]-10
    robot.MoveJ(joint,0,0)
    n=n+1
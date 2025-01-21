from fairino import Robot
import time
# A connection is established with the robot controller. A successful connection returns a robot object
robot = Robot.RPC('192.168.58.2')


def TestSingularAvoidEArc(self):
    """圆弧运动肘关节奇异位姿保护"""
    startdescPose = [-57.170, -690.147, 370.969, 176.438, -8.320, 169.881]
    startjointPos = [78.017, -62.036, 69.561, -94.199, -98.416, -1.360]

    middescPose = [-71.044, -743.395, 375.996, -179.499, -5.398, 168.739]
    midjointPos = [77.417, -55.000, 58.732, -94.360, -95.385, -1.376]

    enddescPose = [-439.979, -512.743, 396.472, 178.112, 3.625, 146.576]
    endjointPos = [40.243, -65.402, 70.802, -92.565, -87.055, -16.465]

    exaxisPos = [0, 0, 0, 0]
    offdese = [0, 0, 0, 0, 0, 0]

    robot.MoveL(desc_pos=startdescPose, tool=0, user=0,vel=50)
    error = robot.SingularAvoidStart(2, 10, 5, 5)
    print("SingularAvoidStart return ", error)
    robot.MoveC(desc_pos_p=middescPose,tool_p=0,user_p=0,desc_pos_t=enddescPose,tool_t=0,user_t=0,vel_p=50,vel_t=50)
    error = robot.SingularAvoidEnd()
    print("SingularAvoidEnd return ", error)

def TestSingularAvoidSArc(self):
    """圆弧运动肩关节奇异位姿保护"""
    startdescPose = [299.993, -168.982, 299.998, 179.999, -0.002, -166.415]
    startjointPos = [-12.160, -71.236, -131.775, -66.992, 90.000, 64.255]

    middescPose = [249.985, -140.988, 299.929, 179.996, -0.013, -166.417]
    midjointPos = [-8.604, -60.474, -137.494, -72.046, 89.999, 67.813]

    enddescPose = [-249.991, -168.980, 299.981, 179.999, 0.004, -107.386]
    endjointPos = [-126.186, -63.401, -136.126, -70.477, 89.998, -108.800]

    exaxisPos = [0, 0, 0, 0]
    offdese = [0, 0, 0, 0, 0, 0]
    robot.SingularAvoidEnd()
    robot.MoveL(desc_pos=startdescPose, tool=0, user=0,vel=50)
    error = robot.SingularAvoidStart(protectMode=2,minShoulderPos=30,minElbowPos=5,minWristPos=5)
    print("SingularAvoidStart return ", error)
    robot.MoveC(desc_pos_p=middescPose,tool_p=0,user_p=0,desc_pos_t=enddescPose,tool_t=0,user_t=0,joint_pos_p=midjointPos,joint_pos_t=endjointPos)
    error = robot.SingularAvoidEnd()
    print("SingularAvoidEnd return ", error)

def TestSingularAvoidWArc(self):
    """圆弧运动腕关节奇异位姿保护"""
    startdescPose = [-352.575, -685.604, 479.380, -15.933, -54.906, 130.699]
    startjointPos = [49.630, -56.597, 60.017, -57.989, 42.725, 146.834]

    middescPose = [-437.302, -372.046, 366.764, -133.489, -62.309, -94.994]
    midjointPos = [21.202, -72.442, 84.164, -51.660, -29.880, 146.823]

    enddescPose = [-653.649, -235.926, 434.525, -176.386, -54.515, -66.734]
    endjointPos = [5.070, -58.920, 55.287, -57.937, -41.207, 146.834]

    exaxisPos = [0, 0, 0, 0]
    offdese = [0, 0, 0, 0, 0, 0]
    robot.MoveL(desc_pos=startdescPose, tool=0, user=0,vel=50)
    # error = robot.SingularAvoidStart(protectMode=2,minShoulderPos=10,minElbowPos=5,minWristPos=4)
    # print("SingularAvoidStart return ", error)
    robot.MoveC(desc_pos_p=middescPose,tool_p=0,user_p=0,desc_pos_t=enddescPose,tool_t=0,user_t=0,vel_p=50,vel_t=50,joint_pos_p=midjointPos,joint_pos_t=endjointPos)
    # error = robot.SingularAvoidEnd()
    # print("SingularAvoidEnd return ", error)

def TestSingularAvoidSLin(self):
    """直线运动肩关节奇异位姿保护"""
    startdescPose = [300.002, -102.991, 299.994, 180.000, -0.001, -166.416]
    startjointPos = [-0.189, -66.345, -134.615, -69.042, 90.000, 76.227]

    enddescPose = [-300.000, -103.001, 299.994, 179.998, 0.003, -107.384]
    endjointPos = [-142.292, -66.345, -134.615, -69.042, 89.997, -124.908]

    exaxisPos = [0, 0, 0, 0]
    offdese = [0, 0, 0, 0, 0, 0]

    robot.MoveL(desc_pos=startdescPose, tool=0, user=0,vel=50)
    # error = robot.SingularAvoidStart(protectMode=2,minShoulderPos=30,minElbowPos=10,minWristPos=3)
    # print("SingularAvoidStart return ", error)
    robot.MoveL(desc_pos=enddescPose, tool=0, user=0,vel=50)
    # error = robot.SingularAvoidEnd()
    # print("SingularAvoidEnd return ", error)

def TestSingularAvoidWLin(self):
    """直线运动腕关节奇异位姿保护"""
    startdescPose = [-352.574, -685.606, 479.415, -15.926, -54.905, 130.693]
    startjointPos = [49.630, -56.597, 60.013, -57.990, 42.725, 146.834]

    enddescPose = [-653.655, -235.943, 434.585, -176.403, -54.513, -66.719]
    endjointPos = [5.072, -58.920, 55.280, -57.939, -41.207, 146.834]

    exaxisPos = [0, 0, 0, 0]
    offdese = [0, 0, 0, 0, 0, 0]

    robot.MoveL(desc_pos=startdescPose, tool=0, user=0,vel=50)
    error = robot.SingularAvoidStart(protectMode=2,minShoulderPos=30,minElbowPos=10,minWristPos=3)
    print("SingularAvoidStart return ", error)
    robot.MoveL(desc_pos=enddescPose, tool=0, user=0,vel=50)
    error = robot.SingularAvoidEnd()
    print("SingularAvoidStart return ", error)

# robot.LoggerInit()
# robot.SetLoggerLevel(1)

# TestSingularAvoidEArc(robot)
# TestSingularAvoidSArc(robot)
# TestSingularAvoidWArc(robot)
# TestSingularAvoidSLin(robot)
TestSingularAvoidWLin(robot)

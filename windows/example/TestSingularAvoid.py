from fairino import Robot
import time
# A connection is established with the robot controller. A successful connection returns a robot object
robot = Robot.RPC('192.168.58.2')


def TestSingularAvoidEArc(self):
    """圆弧运动肘关节奇异位姿保护"""
    startdescPose = [-352.437, -88.350, 226.471, 177.222, 4.924, 86.631]
    startjointPos = [-3.463, -84.308, 105.579, -108.475, -85.087, -0.334]

    middescPose = [-518.339, -23.706, 207.899, -178.420, 0.171, 71.697]
    midjointPos = [-8.587, -51.805, 64.914, -104.695, -90.099, 9.718]

    enddescPose = [-273.934, 323.003, 227.224, 176.398, 2.783, 66.064]
    endjointPos = [-63.460, -71.228, 88.068, -102.291, -90.149, -39.605]

    exaxisPos = [0, 0, 0, 0]
    offdese = [0, 0, 0, 0, 0, 0]

    robot.MoveL(desc_pos=startdescPose, tool=0, user=0,vel=50)
    error = robot.SingularAvoidStart(1,100,50,10)
    print("SingularAvoidStart return ", error)
    robot.MoveC(desc_pos_p=middescPose,tool_p=0,user_p=0,desc_pos_t=enddescPose,tool_t=0,user_t=0,vel_p=50,vel_t=50)
    error = robot.SingularAvoidEnd()
    print("SingularAvoidEnd return ", error)

def TestSingularAvoidSArc(self):
    """圆弧运动肩关节奇异位姿保护"""
    startdescPose = [-379.749, -113.569, 262.288, -178.716, 2.620, 91.597]
    startjointPos = [1.208, -80.436, 93.788, -104.620, -87.372, -0.331]

    middescPose = [-151.941, -155.742, 262.756, 177.693, 2.571, 106.941]
    midjointPos = [16.727, -121.385, 124.147, -90.442, -87.440, -0.318]

    enddescPose = [-211.982, 218.852, 280.712, 176.819, -4.408, 26.857]
    endjointPos = [-63.754, -98.766, 105.961, -94.052, -94.435, -0.366]

    exaxisPos = [0, 0, 0, 0]
    offdese = [0, 0, 0, 0, 0, 0]
    robot.SingularAvoidEnd()
    robot.MoveL(desc_pos=startdescPose, tool=0, user=0,vel=50)
    error = robot.SingularAvoidStart(protectMode=0,minShoulderPos=150,minElbowPos=50,minWristPos=20)
    print("SingularAvoidStart return ", error)
    robot.MoveC(desc_pos_p=middescPose,tool_p=0,user_p=0,desc_pos_t=enddescPose,tool_t=0,user_t=0,vel_p=50,vel_t=50)
    error = robot.SingularAvoidEnd()
    print("SingularAvoidEnd return ", error)

def TestSingularAvoidWArc(self):
    """圆弧运动腕关节奇异位姿保护"""
    startdescPose = [-352.794, -164.582, 132.122, 176.136, 50.177, 85.343]
    startjointPos = [-2.048, -66.683, 121.240, -141.651, -39.776, -0.564]

    middescPose = [-352.353, -3.338, 299.600, -1.730, 58.744, -136.276]
    midjointPos = [-30.807, -92.341, 126.259, -102.944, 33.740, -25.798]

    enddescPose = [-352.353, -3.337, 353.164, -1.729, 58.744, -136.276]
    endjointPos = [-30.807, -98.084, 116.943, -87.886, 33.740, -25.798]

    descPose = [-402.473, -185.876, 103.985, -175.367, 59.682, 94.221]
    jointPos = [-0.095, -50.828, 109.737, -150.708, -30.225, -0.623]

    exaxisPos = [0, 0, 0, 0]
    offdese = [0, 0, 0, 0, 0, 0]
    robot.MoveL(desc_pos=startdescPose, tool=0, user=0,vel=50)
    error = robot.SingularAvoidStart(protectMode=0,minShoulderPos=150,minElbowPos=50,minWristPos=20)
    print("SingularAvoidStart return ", error)
    robot.MoveC(desc_pos_p=middescPose,tool_p=0,user_p=0,desc_pos_t=enddescPose,tool_t=0,user_t=0,vel_p=50,vel_t=50)
    robot.MoveL(desc_pos=descPose, tool=0, user=0,vel=50)
    error = robot.SingularAvoidEnd()
    print("SingularAvoidEnd return ", error)

def TestSingularAvoidSLin(self):
    """直线运动肩关节奇异位姿保护"""
    startdescPose = [-379.749, -113.569, 262.293, -178.715, 2.620, 91.597]
    startjointPos = [1.208, -80.436, 93.788, -104.620, -87.372, -0.331]

    enddescPose = [252.972, -74.287, 316.795, -177.588, 2.451, 97.588]
    endjointPos = [7.165, -170.868, 63.507, 14.965, -87.534, -0.319]

    exaxisPos = [0, 0, 0, 0]
    offdese = [0, 0, 0, 0, 0, 0]

    robot.MoveL(desc_pos=startdescPose, tool=0, user=0,vel=50)
    error = robot.SingularAvoidStart(protectMode=0,minShoulderPos=150,minElbowPos=50,minWristPos=20)
    print("SingularAvoidStart return ", error)
    robot.MoveL(desc_pos=enddescPose, tool=0, user=0,vel=50)
    error = robot.SingularAvoidEnd()
    print("SingularAvoidEnd return ", error)

def TestSingularAvoidWLin(self):
    """直线运动腕关节奇异位姿保护"""
    startdescPose = [-402.473, -185.876, 103.985, -175.367, 59.682, 94.221]
    startjointPos = [-0.095, -50.828, 109.737, -150.708, -30.225, -0.623]

    enddescPose = [-399.264, -184.434, 296.022, -4.402, 58.061, -94.161]
    endjointPos = [-0.095, -65.547, 105.145, -131.397, 31.851, -0.622]

    exaxisPos = [0, 0, 0, 0]
    offdese = [0, 0, 0, 0, 0, 0]

    robot.MoveL(desc_pos=startdescPose, tool=0, user=0,vel=50)
    error = robot.SingularAvoidStart(protectMode=0,minShoulderPos=150,minElbowPos=50,minWristPos=20)
    print("SingularAvoidStart return ", error)
    robot.MoveL(desc_pos=enddescPose, tool=0, user=0,vel=50)
    error = robot.SingularAvoidEnd()
    print("SingularAvoidStart return ", error)

# TestSingularAvoidEArc(robot)
# TestSingularAvoidSArc(robot)
# TestSingularAvoidWArc(robot)
# TestSingularAvoidSLin(robot)
TestSingularAvoidWLin(robot)
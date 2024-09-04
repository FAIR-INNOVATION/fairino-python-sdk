from fairino import Robot
import time
# Establishes a connection with the robot controller and returns a robot object if the connection is successful

robot = Robot.RPC('192.168.58.2')

flag = 1 #switch, 0-close;1-open
delaytime=0 #delay time，ms
isLeftRight=0 #Left and right deviation compensation 0-close，1-open
klr = 0.06 #Left and right adjustment coefficient(sensitivity))
tStartLr = 5 #Left and right start compensation time cyc
stepMaxLr =5 #Left and right maximum compensation amount for each time mm
sumMaxLr = 300 #Maximum compensation amount for left and right mm
isUpLow = 1 #Upper and lower deviation compensation
kud =-0.06 #Upper and lower adjustment coefficient(sensitivity)
tStartUd = 5 #Upper and lower start compensation time cyc
stepMaxUd = 5 #Maximum compensation per up and down mm
sumMaxUd = 300 #Maximum compensation for upper and lower totals
axisSelect = 1 #Upper and lower coordinate system selection, 0-swing; 1-tool; 2-base
referenceType = 0 #Upper and lower reference current Set method, 0-feedback: 1-constant
referSampleStartUd = 4 # Upper and lower reference current sampling starts counting (feedback)，cyc
referSampleCountUd = 1 #  Upper and lower reference current sampling cycle count (feedback)，cyc
referenceCurrent = 10 # Upper and lower reference current mA

startdescPose = [-583.168, 325.637, 1.176, 75.262, 0.978, -3.571]
startjointPos = [-49.049, -77.203, 136.826, -189.074, -79.407, -11.811]
enddescPose = [-559.439, 420.491, 32.252, 77.745, 1.460, -10.130]
endjointPos = [-54.986, -77.639, 131.865, -185.707, -80.916, -12.218]

error = robot.WeldingSetCurrent(1, 230, 0)
print("WeldingSetCurrent return:",error)
robot.WeldingSetVoltage(1, 24, 0)

print("WeldingSetVoltage return:",error)
robot.ArcWeldTraceExtAIChannelConfig(0)
print("ArcWeldTraceExtAIChannelConfig return:",error)

robot.MoveJ(startjointPos,13,0,desc_pos=startdescPose,vel =5)
print("MoveJ return:",error)

error = robot.ArcWeldTraceControl(flag,delaytime, isLeftRight, klr, tStartLr, stepMaxLr, sumMaxLr, isUpLow, kud, tStartUd, stepMaxUd,
                            sumMaxUd, axisSelect, referenceType, referSampleStartUd, referSampleCountUd, referenceCurrent)
print("WireSearchStart return:",error)

robot.ARCStart(1, 0, 10000)
print("ARCStart return:",error)

robot.MoveL(enddescPose,13,0,joint_pos=endjointPos,vel =5)
print("MoveJ return:",error)

robot.ARCEnd(1, 0, 10000)
print("ARCEnd return:",error)

flag = 0
error = robot.ArcWeldTraceControl(flag,delaytime, isLeftRight, klr, tStartLr, stepMaxLr, sumMaxLr, isUpLow, kud, tStartUd, stepMaxUd,
                            sumMaxUd, axisSelect, referenceType, referSampleStartUd, referSampleCountUd, referenceCurrent)
print("WireSearchStart return:",error)

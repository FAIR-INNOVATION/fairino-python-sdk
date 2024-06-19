from fairino import Robot
import time

# A connection is established with the robot controller. A successful connection returns a robot object
robot = Robot.RPC('192.168.58.2')

weldIOType =0
arcNum =0
weldTimeout=5000
weaveNum =0
tool =1
user =0
weaveType = 0
weaveFraquency = 1
weavelncStayTime = 0
weaveRange = 10
weaveLeftStayTime = 10
weaveRightStayTime = 10
weaveCircleRadio =0
weaveStationary =1


#Welding Start
ret = robot.ARCStart(weldIOType,arcNum,weldTimeout)
print("ARCStart", ret)
time.sleep(3)

#Welding end
ret = robot.ARCEnd(weldIOType,arcNum,weldTimeout)
print("ARCEnd", ret)
time.sleep(3)

#Set the relationship between welding current and output analog
ret = robot.WeldingSetCurrentRelation(0,400,0,10)
print("WeldingSetCurrentRelation", ret)
time.sleep(1)

#Obtain the corresponding relationship between welding current and output analog
ret = robot.WeldingGetCurrentRelation()
print("WeldingGetCurrentRelation", ret)
time.sleep(1)

#Set the relationship between welding voltage and output analog
ret = robot.WeldingSetVoltageRelation(0,400,0,10)
print("WeldingSetVoltageRelation", ret)
time.sleep(1)

#The corresponding relationship between welding voltage and output analog is obtained
ret = robot.WeldingGetVoltageRelation()
print("WeldingGetVoltageRelation", ret)
time.sleep(1)

#Set welding current
ret = robot.WeldingSetCurrent(weldIOType,100,0)
print("WeldingSetCurrent", ret)
time.sleep(1)

#Set welding voltage
ret = robot.WeldingSetVoltage(weldIOType,19,1)
print("WeldingSetVoltage", ret)
time.sleep(1)

#Set weave parameters
ret = robot.WeaveSetPara(weaveNum,weaveType,weaveFraquency,weavelncStayTime,weaveRange,weaveLeftStayTime,weaveRightStayTime,weaveCircleRadio,weaveStationary)
print("WeaveSetPara", ret)
time.sleep(1)

# Weave Star
ret = robot.WeaveStart(0)
print("WeaveStart", ret)
time.sleep(1)

ret,pose =robot.GetActualTCPPose(1)
print(ret,pose)
pose[2]=pose[2]+50

ret = robot.MoveL(pose,tool,user)
print("MoveL", ret)
time.sleep(1)

#Set weave parameters in real time
ret = robot.WeaveOnlineSetPara (weaveNum,weaveType,weaveFraquency,weavelncStayTime,weaveRange,0,0,0,weaveLeftStayTime,weaveRightStayTime,weaveCircleRadio,weaveStationary)
print("WeaveOnlineSetPara ", ret)
time.sleep(1)


#Weave end
ret = robot.WeaveEnd(0)
print("WeaveEnd", ret)
time.sleep(1)

#Forward wire feed
ret = robot.SetForwardWireFeed(weldIOType,1)
print("SetForwardWireFeed", ret)
time.sleep(1)

ret = robot.SetForwardWireFeed(weldIOType,0)
print("SetForwardWireFeed", ret)
time.sleep(1)

#Reverse wire feed
ret = robot.SetReverseWireFeed(weldIOType,1)
print("SetReverseWireFeed", ret)
time.sleep(1)
ret = robot.SetReverseWireFeed(weldIOType,0)
print("SetReverseWireFeed", ret)
time.sleep(1)

#Aspirated
ret = robot.SetAspirated(weldIOType,1)
print("SetAspirated", ret)
time.sleep(1)
ret = robot.SetAspirated(weldIOType,0)
print("SetAspirated", ret)
time.sleep(1)

start_desc=[0,0,0,0,0,0]
end_desc=[0,0,0,0,0,0]
start_joint=[0,0,0,0,0,0]
end_joint=[0,0,0,0,0,0]
ret,start_desc =robot.GetActualTCPPose(1)
print("start_desc",start_desc)
ret,end_desc =robot.GetActualTCPPose(1)
end_desc[1]=end_desc[1]+200
print("start_desc",start_desc)
print("end_desc",end_desc)
ret,start_joint=robot.GetInverseKin(0,start_desc)
ret,end_joint=robot.GetInverseKin(0,end_desc)
print("start_joint",start_joint)
print("end_joint",end_joint)

weldLength =40
noweldLength =40

weaveType = 0
weaveFraquency = 1
weavelncStayTime = 0
weaveRange = 10
weaveLeftStayTime = 10
weaveRightStayTime = 10
weaveCircleRadio =0
weaveStationary =1

# Segment Weld Start
ret = robot.SegmentWeldStart(start_desc,end_desc,start_joint,end_joint,weldLength,noweldLength,weldIOType,arcNum,weldTimeout,True,weaveNum,tool,user)
print("SegmentWeldStart", ret)


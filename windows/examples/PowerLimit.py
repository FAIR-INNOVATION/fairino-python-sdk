from fairino import Robot
import time
# Establishes a connection with the robot controller and returns a robot object if the connection is successful

robot = Robot.RPC('192.168.57.2')

# error = robot.SetPowerLimit(0,2)
# print("SetPowerLimit return:",error)

error = robot.DragTeachSwitch(1)
print("DragTeachSwitch return:",error)

error,joint_torque = robot.GetJointTorques()
print("机器人当前关节力矩",error,joint_torque)
# joint_torque = [joint_torque[0],joint_torque[1],joint_torque[2],joint_torque[3],joint_torque[4],joint_torque[5]]
# error_joint = 0
# count =100
# error = robot.ServoJTStart()    #servoJT开始
# print("伺服运动开始错误码",error)
# while(count):
#     if error!=0:
#         error_joint =error
#     joint_torque[0] = joint_torque[0] + 10  #每次1轴增加0.1NM，运动100次
#     error = robot.ServoJT(joint_torque, 0.001)  # 关节空间伺服模式运动
#     count = count - 1
#     time.sleep(0.001)
# print("关节空间伺服模式运动错误码",error_joint)
# error = robot.ServoJTEnd()  #伺服运动结束
# time.sleep(1)
# print("伺服运动结束错误码",error)


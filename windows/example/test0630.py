from fairino import Robot
import time
# A connection is established with the robot controller. A successful connection returns a robot object
import time
import threading

# 与机器人控制器建立连接，连接成功返回一个机器人对象
robot = Robot.RPC('192.168.58.2')

import time

def test(self):
    robot.LinArcFIRPlanningStart(2000, 10000, 720, 1440)
    robot.MoveL(desc_pos=[-331.581, -462.334, 225.274, -173.501, 3.161, 115.864],tool= 0,user=
          0,vel= 100,acc= 100,ovl= 100,blendR= -1,exaxis_pos=[0.0, 0.0, 0.0, 0.0],offset_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],overSpeedStrategy=0,speedPercent=0)
    robot.MoveL(desc_pos=[-219.871, -819.093, 124.722, -163.475, 6.333, 140.797],tool= 0,user=
          0,vel= 100,acc= 100,ovl= 100,blendR= 100,exaxis_pos=[0.0, 0.0, 0.0, 0.0],offset_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],overSpeedStrategy=0,speedPercent=0)
    robot.MoveL(desc_pos=[183.319, -826.070, 70.807, -171.844, -11.320, 167.645],tool= 0,user=
          0,vel= 100,acc= 100,ovl= 100,blendR= 100,exaxis_pos=[0.0, 0.0, 0.0, 0.0],offset_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],overSpeedStrategy=0,speedPercent=0)
    robot.LinArcFIRPlanningEnd()

    robot.PtpFIRPlanningStart(240, 1200)
    robot.MoveJ(joint_pos=[43.849, -71.535, 109.564, -135.187, -89.016, 18.225],tool= 0,user=
          0,vel= 100,acc= 100,ovl= 100,exaxis_pos=[0.0, 0.0, 0.0, 0.0],blendT= -1,offset_flag=0, offset_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    robot.MoveJ(joint_pos=[67.950, -31.106, 58.618, -135.151, -89.017, 18.226],tool= 0,user=
          0,vel= 100,acc= 100,ovl= 100,exaxis_pos=[0.0, 0.0, 0.0, 0.0],blendT= 200,offset_flag=0, offset_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    robot.MoveJ(joint_pos=[105.694, -125.732, 124.263, -105.860, -90.554, 18.230],tool= 0,user=
          0,vel= 100,acc= 100,ovl= 100,exaxis_pos=[0.0, 0.0, 0.0, 0.0],blendT= 200,offset_flag=0, offset_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    robot.PtpFIRPlanningEnd()

    robot.LinArcFIRPlanningStart(2000, 10000, 720, 1440)
    robot.MoveL(desc_pos=[138.430, -103.926, 135.390, -120.507, -116.912, 18.198],tool= 0,user=
                0,vel= 100,acc= 100,ovl= 100,blendR= 50,exaxis_pos=[0.0, 0.0, 0.0, 0.0],offset_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],overSpeedStrategy=0,speedPercent=0)
    robot.MoveC(desc_pos_p=[380.357, -498.600, 323.600, -163.066, -22.643, -171.300],tool_p=0,user_p= 0,vel_p= 100,desc_pos_t=[-171.581, -671.727, 192.097, -170.274, -25.085, 140.438],tool_t= 0,user_t= 0,vel_t= 100)
    robot.LinArcFIRPlanningEnd()

    robot.LinArcFIRPlanningStart(2000, 10000, 720, 1440)
    robot.MoveJ(joint_pos=[138.430, -103.926, 135.390, -120.507, -116.912, 18.198],tool= 0,user=
                0,vel= 100,acc= 100,ovl= 100,exaxis_pos=[0.0, 0.0, 0.0, 0.0],blendT= -1,offset_flag=0, offset_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    robot.MoveC(desc_pos_p=[380.357, -498.600, 323.600, -163.066, -22.643, -171.300],
          tool_p=0,user_p= 0,vel_p= 100,desc_pos_t=[-171.581, -671.727, 192.097, -170.274, -25.085, 140.438],tool_t= 0,user_t= 0,vel_t= 100,ovl=100.0, blendR=120)
    robot.MoveC(desc_pos_p=[-305.647, -317.052, 409.820, 169.616, -30.178, 117.509],tool_p=0,user_p= 0,vel_p= 100,desc_pos_t=[150.549, -235.789, 334.164, 163.763, -31.210, -167.182],tool_t= 0,user_t= 0,vel_t= 100)
    robot.LinArcFIRPlanningEnd()

    robot.LinArcFIRPlanningStart(2000, 10000, 720, 1440)
    robot.MoveL(desc_pos=[288.379, -179.924, 267.471, -171.989, -25.794,-151.376],tool= 0,user=0,
                vel= 100,acc= 100,ovl= 100,blendR= -1,exaxis_pos=[0.0, 0.0, 0.0, 0.0],offset_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],overSpeedStrategy=0,speedPercent=0)
    robot.MoveC(desc_pos_p=[380.357, -498.600, 323.600, -163.066, -22.643, -171.300],
          tool_p=0,user_p= 0,vel_p= 100,desc_pos_t=[-171.581, -671.727, 192.097, -170.274, -25.085, 140.438],tool_t= 0,user_t= 0,vel_t= 100,ovl=100.0, blendR=100)
    robot.MoveL(desc_pos=[-305.647, -317.052, 409.820, 169.616, -30.178, 117.509],tool = 0, user =
          0, vel = 100, acc = 100, ovl = 100, blendR = -1,exaxis_pos=[0.0, 0.0, 0.0, 0.0],offset_pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],overSpeedStrategy=0,speedPercent=0)
    robot.LinArcFIRPlanningEnd()

test(robot)
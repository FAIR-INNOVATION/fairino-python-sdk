from fairino import Robot
import time
# Establishes a connection with the robot controller and returns a robot object if the connection is successful

robot = Robot.RPC('192.168.58.2')

error = robot.SetAnticollision(0,[1, 1, 1, 1, 1, 1],0)
print("SetAnticollision return:",error)

error = robot.SetCollisionDetectionMethod(0)
print("SetCollisionDetectionMethod return:",error)

error = robot.SetStaticCollisionOnOff(1)
print("SetStaticCollisionOnOFF return:",error)

time.sleep(10)

error = robot.SetStaticCollisionOnOff(0)
print("SetStaticCollisionOnOFF return:",error)


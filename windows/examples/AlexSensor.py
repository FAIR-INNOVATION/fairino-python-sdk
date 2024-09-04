from fairino import Robot
import time
# Establishes a connection with the robot controller and returns a robot object if the connection is successful

robot = Robot.RPC('192.168.58.2')
error = robot.AxleSensorConfig(18,0,0,0)
print("AxleSensorConfig return:", error)

error = robot.AxleSensorConfigGet()
print("AxleSensorConfigGet return:", error)

error = robot.AxleSensorActivate(0)
print("AxleSensorActivate return:", error)
time.sleep(1)
error = robot.AxleSensorActivate(1)
print("AxleSensorActivate return:", error)

while(1):
    error = robot.AxleSensorRegWrite(1, 4, 6, 1, 0, 0, 0)
    print("AxleSensorRegWrite return:", error)

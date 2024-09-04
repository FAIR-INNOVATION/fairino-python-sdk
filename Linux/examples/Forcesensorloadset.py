from fairino import Robot
import time
# Establishes a connection with the robot controller and returns a robot object if the connection is successful

robot = Robot.RPC('192.168.58.2')

error = robot.SetForceSensorPayload(0.8)
print("SetForceSensorPayload return:",error)

error = robot.SetForceSensorPayloadCog(0.5,0.6,12.5)
print("SetForceSensorPayLoadCog return:",error)


error = robot.GetForceSensorPayload()
print("GetForceSensorPayLoad return:",error)

error = robot.GetForceSensorPayloadCog()
print("GetForceSensorPayLoadCog return:",error)


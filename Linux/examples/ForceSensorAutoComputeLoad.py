from fairino import Robot
# Establishes a connection with the robot controller and returns a robot object if the connection is successful

robot = Robot.RPC('192.168.58.2')

error = robot.SetForceSensorPayload(0)
print("SetForceSensorPayload return:",error)

error = robot.SetForceSensorPayloadCog(0,0,0)
print("SetForceSensorPayLoadCog return:",error)

error = robot.ForceSensorAutoComputeLoad()
print("ForceSensorAutoComputeLoad return:",error)
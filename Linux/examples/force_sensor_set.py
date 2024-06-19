from fairino import Robot
import time
# A connection is established with the robot controller. A successful connection returns a robot object
robot = Robot.RPC('192.168.58.2')

company = 17    #Sensor manufacturer,17-Kunwei Technology,
device = 0      #Sensor equipment number

error = robot.FT_SetConfig(company, device)   # Force sensor configuration
print("Force sensor configuration",error)
config = robot.FT_GetConfig() # Obtain force sensor configuration
print(' Obtain force sensor configuration ',config)

time.sleep(1)
error = robot.FT_Activate(0)  # Reset force sensor
print("Reset force sensor ",error)
time.sleep(1)
error = robot.FT_Activate(1)  # Activate force sensor
print("Activate force sensor ",error)
time.sleep(1)
error = robot.SetLoadWeight(0.0)    #The end load is set to zero
print("The end load is set to zero",error)
time.sleep(1)
error = robot.SetLoadCoord(0.0,0.0,0.0)  #The end load centroid is set to zero
print("The end load centroid is set to zero",error)
time.sleep(1)
error = robot.FT_SetZero(0)   # Sensor zero removal
print("Sensor zero removal ",error)
error = robot.FT_SetZero(1)   # The zero point of the sensor should be corrected. Please note that no tool can be installed at the end of the sensor.
print("Sensor zero corrected ",error)

error = robot.FT_GetForceTorqueRCS()  # Obtain force/torque data in the reference coordinate system
print("Obtain force/torque data in the reference coordinate system ",error)
error = robot.FT_GetForceTorqueOrigin()   # Obtain raw force/torque data from the force sensor
print("Obtain raw force/torque data from the force sensor ",error)



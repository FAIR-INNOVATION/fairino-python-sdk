from fairino import Robot
import time
# Establishes a connection with the robot controller and returns a robot object if the connection is successful

robot = Robot.RPC('192.168.58.2')

id = 1 #Welding process number(1-99)
startCurrent = 177 #Arc starting current(A)
startVoltage = 27 #Arc starting voltage(V)
startTime = 1000 #Arc starting time(ms)
weldCurrent = 178 #Welding current(A)
weldVoltage = 28 #Welding voltage(V)
endCurrent = 176 #Arc ending current(A)
endVoltage = 26 # Arc ending voltage(V)
endTime = 1000 #Arc ending time(ms)

error = robot.WeldingSetProcessParam(id, startCurrent, startVoltage, startTime, weldCurrent, weldVoltage,
                                          endCurrent, endVoltage, endTime)

print("WeldingSetProcessParam return:",error)

error = robot.WeldingGetProcessParam(1)
print("WeldingGetProcessParam return:",error)


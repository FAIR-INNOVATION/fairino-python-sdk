from fairino import Robot
import time
# Establishes a connection with the robot controller and returns a robot object if the connection is successful

robot = Robot.RPC('192.168.58.2')

error = robot.SetAuxDO(1,True,False,False)
print("SetAuxDO 1  return:",error)

error = robot.SetAuxDO(3,True,False,False)
print("SetAuxDO 3  return:",error)

error = robot.SetAuxAO(0,10,False)
print("SetAuxAO 0   return:",error)

error = robot.SetAuxAO(1,87,False)
print("SetAuxAO 1  return:",error)

error = robot.SetOutputResetExtDO(1)
print("SetOutputResetExtDO return:",error)

error = robot.SetOutputResetExtAO(1)
print("SetOutputResetExtAO return:",error)

error = robot.ProgramRun()
print("ProgramRun return:",error)
time.sleep(3)
error = robot.ProgramStop()
print("ProgramPause return:",error)

time.sleep(3)
error = robot.SetAuxDO(1,True,False,False)
print("SetAuxDO 1  return:",error)

error = robot.SetAuxDO(3,True,False,False)
print("SetAuxDO 3  return:",error)

error = robot.SetAuxAO(0,10,False)
print("SetAuxAO 0   return:",error)

error = robot.SetAuxAO(1,87,False)
print("SetAuxAO 1  return:",error)

error = robot.SetOutputResetExtDO(0)
print("SetOutputResetExtDO return:",error)

error = robot.SetOutputResetExtAO(0)
print("SetOutputResetExtAO return:",error)

error = robot.ProgramRun()
print("ProgramRun return:",error)
time.sleep(3)
error = robot.ProgramStop()
print("ProgramPause return:",error)





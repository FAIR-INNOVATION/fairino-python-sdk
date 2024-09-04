from fairino import Robot
import time
# Establishes a connection with the robot controller and returns a robot object if the connection is successful

robot = Robot.RPC('192.168.58.2')
time.sleep(5)
error = robot.SetDO(1,1)
print("SetDO 1  return:",error)

error = robot.SetDO(3,1)
print("SetDO 3  return:",error)

error = robot.SetToolDO(1,1)
print("SetToolDO return:",error)

error = robot.SetAO(0,25)
print("SetAO 0   return:",error)

error = robot.SetAO(1,87)
print("SetAO 1  return:",error)

error = robot.SetToolAO(0,54)
print("SetToolAO return:",error)

error = robot.SetOutputResetCtlBoxDO(1)
print("SetOutputResetCtlBoxDO return:",error)

error = robot.SetOutputResetCtlBoxAO(1)
print("SetOutputResetCtlBoxAO return:",error)

error = robot.SetOutputResetAxleDO(1)
print("SetOutputResetCtlBoxDO return:",error)

error = robot.SetOutputResetAxleAO(1)
print("SetOutputResetCtlBoxAO return:",error)

error = robot.ProgramRun()
print("ProgramRun return:",error)
time.sleep(3)
error = robot.ProgramStop()
print("ProgramPause return:",error)

time.sleep(5)

error = robot.SetDO(1,1)
print("SetDO 1  return:",error)

error = robot.SetDO(3,1)
print("SetDO 3  return:",error)

error = robot.SetToolDO(1,1)
print("SetToolDO return:",error)

error = robot.SetAO(0,25)
print("SetAO 0   return:",error)

error = robot.SetAO(1,87)
print("SetAO 1  return:",error)

error = robot.SetToolAO(0,54)
print("SetToolAO return:",error)
error = robot.SetOutputResetCtlBoxDO(0)
print("SetOutputResetCtlBoxDO return:",error)

error = robot.SetOutputResetCtlBoxAO(0)
print("SetOutputResetCtlBoxAO return:",error)

error = robot.SetOutputResetAxleDO(0)
print("SetOutputResetCtlBoxDO return:",error)

error = robot.SetOutputResetAxleAO(0)
print("SetOutputResetCtlBoxAO return:",error)

error = robot.ProgramRun()
print("ProgramRun return:",error)
time.sleep(3)
error = robot.ProgramStop()
print("ProgramPause return:",error)
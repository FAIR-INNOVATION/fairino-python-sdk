from fairino import Robot
import time
# A connection is established with the robot controller. A successful connection returns a robot object
robot = Robot.RPC('192.168.58.2')
def print_program_state():
    pstate = robot.GetProgramState()    # Obtain the execution status of robot job programs
    linenum = robot.GetCurrentLine()    # Obtain the execution line number of the current robot job program
    name = robot.GetLoadedProgram()     # Obtain the name of the loaded job program
    print("the robot program state is:",pstate[1])
    print("the robot program line number is:",linenum[1])
    print("the robot program name is:",name[1])
    time.sleep(1)
robot.Mode(0)
print_program_state()
ret = robot.ProgramLoad('/fruser/test0923.lua')   # Load the specified job program
print("Load the specified job program ", ret)
ret = robot.ProgramRun()     # Run the currently loaded job program
print("Run the currently loaded job program ", ret)
time.sleep(2)
print_program_state()
ret = robot.ProgramPause()   # Pause the currently running job program
print("Pause the currently running job program ", ret)
time.sleep(2)
print_program_state()
ret = robot.ProgramResume()  # Resume the currently paused job program
print("Resume the currently paused job program ", ret)
time.sleep(2)
print_program_state()
ret = robot.ProgramStop()    # Terminate the currently running job program
print("Terminate the currently running job program ", ret)
time.sleep(2)
print_program_state()

flag = 1
ret = robot.LoadDefaultProgConfig(flag,'/fruser/testPTP.lua')    # Set up and automatically load the default operating program
print("Set up and automatically load the default operating program ", ret)


from fairino import Robot
import time
# A connection is established with the robot controller. A successful connection returns a robot object
robot = Robot.RPC('192.168.58.2')
type = 1
name = 'tpd2023'
period = 4
di = 0
do = 0
robot.Mode(1)  # Robot goes into manual mode
time.sleep(1)
robot.DragTeachSwitch(1)  # The robot cuts into the drag teaching mode
time.sleep(1)
ret = robot.SetTPDStart(name, period, do_choose=do)   # Start trajectory recording
print("Start trajectory recording", ret)
time.sleep(15)
ret = robot.SetWebTPDStop()  # Stop trajectory recording
print("Stop trajectory recording", ret)
robot.DragTeachSwitch(0)  # The robot exits drag teaching mode


# robot.SetTPDDelete('tpd2023')   # Delete trajectory record

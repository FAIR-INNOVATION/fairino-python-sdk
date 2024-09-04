from fairino import Robot
# Establishes a connection with the robot controller and returns a robot object if the connection is successful

robot = Robot.RPC('192.168.58.2')

#Extended IO-Configure welding machine gas detection signal
error = robot.SetAirControlExtDoNum(10)
print("SetAirControlExtDoNum 10 return:",error)

#Extended IO-Configure welding machine arc starting signa
error = robot.SetArcStartExtDoNum(11)
print("SetArcStartExtDoNum 11 return:",error)

#Extended IO-Configure welding machine reverse wire feeding signal
error = robot.SetWireReverseFeedExtDoNum(12)
print("SetWireReverseFeedExtDoNum 12 return:",error)

#Extended IO-Configure welding machine forward wire feeding signal
error = robot.SetWireForwardFeedExtDoNum(13)
print("SetWireForwardFeedExtDoNum 13 return:",error)

#Extended IO-Configure welding machine arc starting success signal
error = robot.SetArcDoneExtDiNum(10)
print("SetArcDoneExtDiNum 10 return:",error)

#Extended IO-Configure welding machine ready signal
error = robot.SetWeldReadyExtDiNum(11)
print("SetWeldReadyExtDiNum 11 return:",error)

#Extended IO-Configure welding interrupt recovery signal
error = robot.SetExtDIWeldBreakOffRecover(12,13)
print("SetExtDIWeldBreakOffRecover 12  13 return:",error)

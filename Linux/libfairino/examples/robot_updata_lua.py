from fairino import Robot

# A connection is established with the robot controller. A successful connection returns a robot object
robot = Robot.RPC('192.168.58.2')
error = robot.PointTableUpdateLua("point_table_a.db","testpoint.lua")
print("PointTableUpdateLua:",error)
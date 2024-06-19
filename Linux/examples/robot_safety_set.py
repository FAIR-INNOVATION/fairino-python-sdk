from fairino import Robot
# A connection is established with the robot controller. A successful connection returns a robot object
robot = Robot.RPC('192.168.58.2')
level = [1.0,2.0,3.0,4.0,5.0,6.0]
error = robot.SetAnticollision(0,level,1)
print("Set collision level:",error)
level = [50.0,20.0,30.0,40.0,50.0,60.0]
error = robot.SetAnticollision(1,level,1)
print("Set collision level:",error)
error = robot.SetCollisionStrategy(1)
print("Set the strategy after collision:",error)

p_limit = [170.0,80.0,150.0,80.0,170.0,160.0]
n_limit = [-170.0,-260.0,-150.0,-260.0,-170.0,-160.0]
error = robot.SetLimitPositive(p_limit)
print("Set positive limit:",error)
error = robot.SetLimitNegative(n_limit)
print("Set negative limit:",error)

error = robot.ResetAllError()
print("Error status cleared:",error)

error = robot.FrictionCompensationOnOff(1)
print("Joint friction compensation switch:",error)

lcoeff = [0.9,0.9,0.9,0.9,0.9,0.9]
wcoeff = [0.4,0.4,0.4,0.4,0.4,0.4]
ccoeff = [0.6,0.6,0.6,0.6,0.6,0.6]
fcoeff = [0.5,0.5,0.5,0.5,0.5,0.5]
error = robot.SetFrictionValue_level(lcoeff)
print("Set joint friction compensation coefficient formal installation:",error)
error = robot.SetFrictionValue_wall(wcoeff)
print("Set joint friction compensation coefficient - Side Mount:",error)
error =robot.SetFrictionValue_ceiling(ccoeff)
print("Set joint friction compensation coefficient-Inverted:",error)
error =robot.SetFrictionValue_freedom(fcoeff)
print("Set joint friction compensation coefficient-free installation:",error)



robot.SetFrictionValue_level(lcoeff)

from fairino import Robot
import time
# A connection is established with the robot controller. A successful connection returns a robot object
from Test1 import ASD
from Test2 import CSD
from Test3 import BSD




if __name__ == "__main__":
    robot = ASD()
    B = BSD(robot)
    B.Mode()
    C = CSD(B)
    C.Mode()
    # C.BSD.Mode()


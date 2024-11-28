from fairino import Robot
import time
# A connection is established with the robot controller. A successful connection returns a robot object
from Test3 import BSD
class CSD:
    def __init__(self,BSD):
        self.BSD = BSD
        print("CCC")
    def Mode(self):
        self.BSD.Mode()







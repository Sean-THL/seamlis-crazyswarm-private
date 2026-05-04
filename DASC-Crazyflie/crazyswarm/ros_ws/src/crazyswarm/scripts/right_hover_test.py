from pycrazyswarm import *
from hover_and_land import *
import numpy as np
from emergency_break import *
from std_msgs.msg import Bool


DES_HEIGHT = 4 # m
Z_SPEED = 1  # m/s


def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    print("Testing FlyLab Right Region Flight Test...")
    hover_all(swarm,HEIGHT = DES_HEIGHT)
    # Each robot implements its own control input u_i  
    while True:          
        for drones in swarm.allcfs.crazyflies:
            drones.cmdVelocityWorld(np.array([0,-0.3,0]), yawRate=0)
        timeHelper.sleep(0.01)



if __name__ == "__main__":
    main()

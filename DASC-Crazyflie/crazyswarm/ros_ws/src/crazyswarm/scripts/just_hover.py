from pycrazyswarm import *
from hover_and_land import *
import numpy as np
from emergency_break import *
from std_msgs.msg import Bool


HEIGHT = 0.75 # m
Z_SPEED = 1  # m/s


def main():
    swarm = Crazyswarm()
    print("Testing hovering...")
    hover_all(swarm)


if __name__ == "__main__":
    main()

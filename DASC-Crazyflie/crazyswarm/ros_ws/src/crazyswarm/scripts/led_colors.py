import numpy as np
from pycrazyswarm import *


def set_LED(swarm, rgb):
    for cf in swarm.allcfs.crazyflies:
        cf.setLEDColor(*rgb)


if __name__ == "__main__":
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    # Configure the CFs so that the LED ring displays the solid color.
    # Overrides the launch file and the firmware default.
    for cf in allcfs.crazyflies:
        cf.setParam("ring/effect", 7)

    TRIALS = 1
    for i in range(TRIALS):
        for rgb in rgb_bits:
            for cf in allcfs.crazyflies:
                cf.setLEDColor(*rgb)
            timeHelper.sleep(2.0)
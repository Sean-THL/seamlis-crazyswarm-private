"""Single CF: takeoff, follow absolute-coords waypoints, land."""

import numpy as np

from pycrazyswarm import Crazyswarm


Z = 0.5
TAKEOFF_DURATION = 2.5
GOTO_DURATION = 3.0
WAYPOINTS = np.array([
    (1.0, 0.0, Z),
    (1.0, 1.0, Z),
    (0.0, 1.0, Z),
    (0.0, 0.0, Z),
])


def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    for cf in swarm.allcfs.crazyflies:
        cf.takeoff(targetHeight=Z, duration=TAKEOFF_DURATION)
    timeHelper.sleep(TAKEOFF_DURATION + 1)
    for p in WAYPOINTS:
        for cf in swarm.allcfs.crazyflies:
            cf.goTo(cf.initialPosition + p, yaw=0.0, duration=GOTO_DURATION)
        timeHelper.sleep(GOTO_DURATION + 1.0)
    for cf in swarm.allcfs.crazyflies:
        cf.land(targetHeight=0.05, duration=TAKEOFF_DURATION)
    timeHelper.sleep(TAKEOFF_DURATION + 1.0)


if __name__ == "__main__":
    main()

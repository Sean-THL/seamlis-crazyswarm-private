from pycrazyswarm import *

LAND_SPEED = 0.5 #m/s
Z_SPEED = 0.75 # m/s
LAND_HEIGHT = 0.04 #m


# Take off and hover. If y is pressed, all drones land
def hover_all(swarm, HEIGHT = 0.7, Z_SPEED = 0.75):
    timeHelper = swarm.timeHelper
    for cf in swarm.allcfs.crazyflies:
        cf.takeoff(targetHeight=HEIGHT, duration=HEIGHT / Z_SPEED)
    timeHelper.sleep(HEIGHT / Z_SPEED)
    user_input = input("Continue? [y/n]")
    if (user_input != "y"):
        print("Performing landing...")
        swarm.land_all()
        exit()

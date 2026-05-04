from pycrazyswarm import Crazyswarm
import numpy as np

height = 0.7
second_height = 7.5
LAND_SPEED = 0.5 #m/s
HOVER_DURATION = 2.0
Z_SPEED = 0.75 # m/s

def land_all(swarm):
    # land all
    timeHelper = swarm.timeHelper
    max_duration = 0.0
    for cf in swarm.allcfs.crazyflies:
        z = cf.position()[2]
        duration = z / Z_SPEED + 1
        max_duration = max(max_duration, duration)
        cf.land(targetHeight=0.04, duration=duration)
    timeHelper.sleep(max_duration)


def M_shape(height):
    M = np.array([
        [1,1,height],
        [1,0,height],
        [1,-1,height],
        [0.35,0.5,height],
        [-1,-1,height],
        [-1,0,height],
        [-1,1,height],
        [-0.35,0.5,height]
    ])

    return M

def rotate(swarm, desired_height, period = 30.0, steps = 10):
    
    timeHelper = swarm.timeHelper
    M = M_shape(desired_height)
    
    # rgb = [(0.0,0.0,1.0),(1.0,1.0,0.0)]
    for i in range(steps+1):
        theta = i * 2*np.pi / steps

        # get rotation matrix
        rot = np.array([
            [np.cos(theta), -np.sin(theta), 0],
            [np.sin(theta), np.cos(theta),0],
            [0,0,1]
            ])

        # send the commanded positions
        for j in range(8):
            swarm.allcfs.crazyflies[j].goTo(M[j] @ rot, 0, period/steps)
        timeHelper.sleep(period/steps)


def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper

    for cf in swarm.allcfs.crazyflies:
        cf.takeoff(targetHeight=height, duration=height / Z_SPEED)
    timeHelper.sleep(height / Z_SPEED)

    user_input = input("Continue? [y/n]")
    if (user_input != "y"):
        land_all(swarm)
        exit()


    duration = 3.0
    M = M_shape(height)
    for (j, cf) in enumerate(swarm.allcfs.crazyflies):
        cf.goTo(M[j], 0, duration)
    timeHelper.sleep(duration)

    duration = (second_height - height) / Z_SPEED
    M = M_shape(second_height)
    for (j, cf) in enumerate(swarm.allcfs.crazyflies):
        cf.goTo(M[j], 0, duration)
    timeHelper.sleep(duration+10)   



    swarm.allcfs.crazyflies[3].goTo(np.array([0.0,-1.0,second_height]), 0, 4.0)
    timeHelper.sleep(4)
    swarm.allcfs.crazyflies[-1].goTo(np.array([0.0,1.0,second_height]), 0, 4.0)
    timeHelper.sleep(4)   
    


    # land all
    land_all(swarm)


if __name__ == "__main__":
    main()



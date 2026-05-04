from pycrazyswarm import Crazyswarm
import numpy as np
import matplotlib.pyplot as plt

height = 0.7 # m 
second_height = 3.0 # m 

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

def rotate(swarm, current_position, right=1):
    theta = 30*right
    rot = np.array([
        [np.cos(theta), -np.sin(theta), 0],
        [np.sin(theta), np.cos(theta),0],
        [0,0,1]
        ])
    period = 5
    timeHelper = swarm.timeHelper
    for j in range(9):
        current_loc = swarm.allcfs.crazyflies[j].position()
        swarm.allcfs.crazyflies[j].goTo(rot @ (current_loc - np.array(current_position)) + np.array(current_position), 0, period)
    timeHelper.sleep(period)
    current_loc = swarm.allcfs.crazyflies[0].position()
    return rot @ (current_loc - np.array(current_position)) + np.array(current_position)




def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper

    # Take off
    for cf in swarm.allcfs.crazyflies:
        cf.takeoff(targetHeight=height, duration=height / Z_SPEED)
    timeHelper.sleep(height / Z_SPEED)

    offset = 0.5
    duration = 2.5
    current_pos = np.array([0.0,0.0,height])
    x_bounds = [-2.5,2.0]
    y_bounds = [-2.0,2.0]
    z_bounds = [0.5, 7.0]

    while True:
        user_input = input("Command: forward (w), backward (s), left (a), right (d), up (e), down (q)")
        command = None
        if user_input == "w":
            if current_pos[0] + offset > x_bounds[1]:
                print("It is too forward - try moving them backward. Current Position: ", current_pos)
                continue
            command = [offset,0,0]
            print("moving forward... Current Position: ", current_pos)
        elif user_input == "s":
            if current_pos[0] - offset < x_bounds[0]:
                print("It is too backward - try moving them forward. Current Position: ", current_pos)
                continue
            command = [-offset,0,0]
            print("moving backward... Current Position: ", current_pos)
        elif user_input == "a":
            if current_pos[1] + offset > y_bounds[1]:
                print("It is too left - try moving them right. Current Position: ", current_pos)
                continue
            command = [0,offset,0]
            print("moving left... Current Position: ", current_pos)
        elif user_input == "d":
            if current_pos[1] - offset < y_bounds[0] and current_pos[2] < 5:
                print("It is too right - try moving them left. Current Position: ", current_pos)
                continue
            command = [0,-offset,0]
            print("moving right... Current Position: ", current_pos)
        elif user_input == "e":
            if current_pos[2] +offset > z_bounds[1]:
                print("It is too high - try moving them down. Current Position: ", current_pos)
                continue
            command = [0,0,offset]
            print("moving up... Current Position: ", current_pos)
        elif user_input == "q":
            if current_pos[2]  -offset < z_bounds[0]:
                print("It is too low - try moving them up. Current Position: ", current_pos)
                continue
            command = [0,0,-offset]
            print("moving down... Current Position: ", current_pos)
        elif user_input == "x":
            add_time = np.linalg.norm([current_pos[0],-current_pos[1]])/0.75
            swarm.allcfs.goTo([-current_pos[0],-current_pos[1],0], 0 , duration+add_time)
            timeHelper.sleep(duration+add_time)
            print("Landing the drones...")
            break
        elif user_input == "1":
            current_pos = rotate(swarm, current_pos)
            print("Rotating the drones in right...")
        elif user_input == "2":
            current_pos = rotate(swarm, current_pos, -1)
            print("Rotating the drones in left...")
        else:
            print("Invalid Input - please try again. Current Position: ", current_pos)
            continue
        if command != None:
            current_pos+=np.array(command)
            swarm.allcfs.goTo(command, 0 , duration)
            timeHelper.sleep(duration)
    land_all(swarm)

if __name__ == "__main__":
    main()
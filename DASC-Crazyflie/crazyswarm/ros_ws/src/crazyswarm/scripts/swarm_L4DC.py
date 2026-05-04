from pycrazyswarm import Crazyswarm
import numpy as np
import matplotlib.pyplot as plt
import hungarian
import cvxpy as cp

Z_HEIGHT = 4.0  # Height at which the Crazyflies will operate
YAW = np.pi/2 # PI if the letters face the entrance of fly lab, PI/2 if viewing from control room/balcony, 0 if viewing from desk opposite the entrance
R = np.array([[np.cos(YAW), np.sin(YAW)],[-np.sin(YAW), np.cos(YAW)]])

L_SHAPE = np.array([
    [1.0, 1.0],
    [0.6, 1.0],
    [0.2, 1.0],
    [-0.2, 1.0],
    [-0.6, 1.0],
    [-1.0, 1.0],
    [-1.0, 0.6],
    [-1.0, 0.2],
    [-1.0, -0.2]
]) @ R

FOUR_SHAPE = np.array([
    [1.0, 0.0],
    [0.5, 0.0],
    [0.0, 0.0],
    [-0.5, 0.0],
    [-1.0, 0.0],
    [0.0, -0.5],
    [0.0, 0.5],
    [0.0, 1.0],
    [0.5, 0.5]
]) @ R

D_SHAPE = np.array([
    [1.0, 1.0],
    [0.33, 1.0],
    [-0.33, 1.0],
    [-1.0, 1.0],
    [-0.8, 0.5],
    [-0.5, 0.0],
    [0.0, -0.2],
    [0.8, 0.5],
    [0.5, 0.0]
]) @ R

C_SHAPE = np.array([
    [0.866, -0.5],
    [1.0, 0.0],
    [0.866, 0.5],
    [0.5, 0.866],
    [0.0, 1.0],
    [-0.5, 0.866],
    [-0.866, 0.5],
    [-1.0, 0.0],
    [-0.866, -0.5]
]) @ R

def compute_optimal_assignment(swarm, goal_positions):
    """
    Computes the optimal assignment of Crazyflies to target positions using the Hungarian algorithm.
    :param swarm: Crazyswarm instance containing all Crazyflies.
    :param positions: Target positions for the Crazyflies.
    :return: List of indices representing the optimal assignment.
    """
    current_positions = get_current_positions(swarm)  # Get the current positions of the Crazyflies
    n = len(swarm.allcfs.crazyflies)
    C = np.zeros((n, n))  # Cost matrix for the assignment problem
    for i in range(len(current_positions)):
        for j in range(n):
            C[i,j] = np.hypot(current_positions[i][0] - goal_positions[j][0],
                              current_positions[i][1] - goal_positions[j][1])

    H = hungarian.Hungarian(C)  # Calculate the optimal assignment using the Hungarian algorithm
    H.calculate()  # Perform the assignment calculation
    return H.get_results()

def get_current_positions(swarm):
    """
    Retrieves the current positions of all Crazyflies in the swarm.
    :param swarm: Crazyswarm instance containing all Crazyflies.
    :return: List of current positions of the Crazyflies.
    """
    return np.array([cf.position() for cf in swarm.allcfs.crazyflies])

def transform_pos(pos, origin, pitch, roll, yaw):
    """
    Transforms a given shape by applying a translation and rotation.
    :param shape: The original position
    :param offset: Translation vector to apply to the shape.
    :param pitch: Pitch angle in radians for rotation.
    :param roll: Roll angle in radians for rotation.
    :param yaw: Yaw angle in radians for rotation.
    :return: Transformed shape as a numpy array of points.
    """
    rotation_matrix = np.array([
        [np.cos(yaw) * np.cos(pitch), np.sin(yaw) * np.cos(pitch), -np.sin(pitch)],
        [-np.sin(yaw) * np.cos(roll) + np.cos(yaw) * np.sin(pitch) * np.sin(roll),
         np.cos(yaw) * np.cos(roll) + np.sin(yaw) * np.sin(pitch) * np.sin(roll),
         np.cos(pitch) * np.sin(roll)],
        [np.sin(yaw) * np.sin(roll) + np.cos(yaw) * np.sin(pitch) * np.cos(roll),
         -np.cos(yaw) * np.sin(roll) + np.sin(yaw) * np.sin(pitch) * np.cos(roll),
         np.cos(pitch) * np.cos(roll)]
    ])
    
    return rotation_matrix @ (pos-origin) + origin

def move(swarm,shape, assignment):
################################## CBF-QP Controller ####################################
    D_MIN = 0.3
    Kp = 1.0
    num_drones = len(swarm.allcfs.crazyflies)
    num_constraints = int((num_drones-1)*(num_drones)/2)
    u1 = cp.Variable((num_drones*3,1))
    u1_des = cp.Parameter((num_drones*3,1),value = np.zeros((num_drones*3,1)) )
    A1 = cp.Parameter((num_constraints,num_drones*3),value=np.zeros((num_constraints,num_drones*3)))
    b1 = cp.Parameter((num_constraints,1),value=np.zeros((num_constraints,1)))
    const1 = [A1 @ u1 >= b1]
    objective1 = cp.Minimize( cp.sum_squares( u1 - u1_des  ))
    cbf_controller = cp.Problem( objective1, const1 )
    #########################################################################################
    current_pos = np.array([drone.position()[:3] for drone in swarm.allcfs.crazyflies])
    error = np.zeros((num_drones, 3))
    for (i, goal) in assignment:
        error[i] = np.append(shape[goal], Z_HEIGHT) - current_pos[i]
    # Solves the central CBF-QP until the drones are close to their initial positions while enforcing collision avoidance
    while np.any(np.linalg.norm(error, axis=1) > 0.05):
        for i in range(num_drones):
            u1_des.value[3*i:3*i+3] = Kp*error[i].reshape(-1,1)
            for j in range(i+1, num_drones):
                h =  np.linalg.norm(current_pos[i] - current_pos[j])**2 - D_MIN**2 
                dh_dxi = 2*(current_pos[i] - current_pos[j]).T
                A1.value[i,3*i:3*i+3] =  dh_dxi[:]
                A1.value[i,3*j:3*j+3] = -dh_dxi[:]
                b1.value[i] = -1.5*h
        cbf_controller.solve(solver = "GUROBI")

        # Implements the control input from CBF-QP
        for i in range(num_drones):
            vec = u1.value[3*i:3*i+3].reshape(1,-1)[0]
            swarm.allcfs.crazyflies[i].cmdVelocityWorld(vec, yawRate=0)
        swarm.timeHelper.sleepForRate(30)
        current_pos = np.array([drone.position()[:3] for drone in swarm.allcfs.crazyflies])
        for (i, goal) in assignment:
            error[i] = np.append(shape[goal], Z_HEIGHT) - current_pos[i]
    # Stops the drones 

    for cf in swarm.allcfs.crazyflies:
        cf.notifySetpointsStop()
    swarm.timeHelper.sleep(0.1)
    for cf in swarm.allcfs.crazyflies:
        cf.goTo(cf.position(), 0, 0.1)
    swarm.timeHelper.sleep(0.1)

def make_shape(swarm, shape, speed=0.5, angle_increment=np.deg2rad(10.0)):
    """
    Makes a shape by commanding all Crazyflies to go to the specified positions.
    :param swarm: Crazyswarm instance containing all Crazyflies.
    :param shape: The shape to be made as a numpy array of points (only x-y coordinates).
    """

    optimal_assignment = compute_optimal_assignment(swarm, shape)     
    duration = max([np.linalg.norm(swarm.allcfs.crazyflies[i].position()[:2] - shape[g]) for (i,g) in optimal_assignment])/speed
    for (cf, pos) in optimal_assignment:
        swarm.allcfs.crazyflies[cf].goTo(shape[pos].tolist() + [Z_HEIGHT], 0, duration)
    
    # dt = 0.001
    # smallest_dist = 1000.0
    # tau = 0.0
    # while tau < duration:

    #     pos_list = [cf.position() for cf in swarm.allcfs.crazyflies]
    #     min_dist = 1000.0
    #     N = len(swarm.allcfs.crazyflies)
    #     for i in range(N):
    #         for j in range(N):
    #             if j > i:
    #                 d = np.linalg.norm(pos_list[i] - pos_list[j])
    #                 min_dist = min(d, min_dist)

    #     if min_dist <= 0.35:
    #         print(tau, min_dist)
        
    #     smallest_dist = min(smallest_dist, min_dist)


    #     tau += dt
    #     swarm.timeHelper.sleep(dt)
        
    # if smallest_dist <= 0.35:
    #     print(f"*** Smallest: {smallest_dist}")

    swarm.timeHelper.sleep(duration)
    # move(swarm, shape, optimal_assignment)

    roll, pitch, yaw = 0.0, 0.0, 0.0
    while True:
        user_input = input("Press c to continue, q/e to yaw, w/s to pitch, a/d to roll, x to land: ")
        if user_input == "x":
            return True
        elif user_input == "c":
            break
        # elif user_input == "e":
        #     yaw += angle_increment
        # elif user_input == "q":
        #     yaw -= angle_increment
        # elif user_input == "w":
        #     pitch -= angle_increment
        # elif user_input == "s":
        #     pitch += angle_increment
        # elif user_input == "a":
        #     roll += angle_increment
        # elif user_input == "d":
        #     roll -= angle_increment
        else:
            continue

        roll = np.clip(roll, -np.pi/6, np.pi/6)
        pitch = np.clip(pitch, -np.pi/6, np.pi/6)
        Kp = 1.0
        
        for (i, pos) in optimal_assignment:
            # error = np.append(shape[pos], Z_HEIGHT) - swarm.allcfs.crazyflies[i].position()
            # vel_cmd = Kp*error
            # swarm.allcfs.crazyflies[i].cmdVelocityWorld(vel_cmd, yaw_rate=0)
            swarm.allcfs.crazyflies[i].goTo(transform_pos(np.append(shape[pos], Z_HEIGHT), np.array([0.0,0.0,Z_HEIGHT]), pitch, roll, yaw), 0, duration)

        swarm.timeHelper.sleep(1.5)
    return False

def land_all(swarm):
    # Land all
    for cf in swarm.allcfs.crazyflies:
        print(cf.position())
        cf.goTo(np.append(cf.position()[:2], -0.75),0.0, 1.5)
    swarm.timeHelper.sleep(1.5)
    # for cf in swarm.allcfs.crazyflies:
    #     cf.stop()


def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper

    # for cf in swarm.allcfs.crazyflies:
    #     cf.enableCollisionAvoidance(swarm.allcfs.crazyflies, [0.2, 0.2, 0.3])

    initial_positions = np.array([cf.position()[:2] for cf in swarm.allcfs.crazyflies])

    # Take off
    for cf in swarm.allcfs.crazyflies:
        cf.takeoff(targetHeight=Z_HEIGHT, duration=1.0)
    timeHelper.sleep(1.0)
    scale = 1.6
    if (input("Continue? [y/n]") == "y"):
        sequence = [scale*L_SHAPE, scale*FOUR_SHAPE, scale*D_SHAPE, scale*C_SHAPE]#, scale*D_SHAPE, scale*FOUR_SHAPE, scale*L_SHAPE, initial_positions]
        for i, shape in enumerate(sequence):
            land = make_shape(swarm, shape, speed=0.25)
            if land:
                break

    land_all(swarm)




if __name__ == "__main__":
    main()



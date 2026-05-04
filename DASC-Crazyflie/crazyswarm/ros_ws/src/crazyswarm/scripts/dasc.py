from pycrazyswarm import Crazyswarm
import numpy as np
from hover_and_land import *
import cvxpy as cp

HEIGHT = 1 #m
LAND_SPEED = 0.5 #m/s
HOVER_DURATION = 2.0
Z_SPEED = 0.75 # m/s
MOVEMENT_DURATION = 7.5


def D():
    return np.array([
        [0, -0.9, 2],
        [0, -0.9, 1.5],
        [0,-0.9, 1.0],
        [0, -0.9, 0.5],
        [0, -0.6, 1.8],
        [0, -0.6, 0.7],
        [0, -0.3, 1.6],
        [0, -0.3, 1.25],
        [0, -0.3, 0.9]
    ])    

def A():
    return np.array([
        [0, -0.5, 2.0],
        [0, -0.625, 1.6],
        [0, -0.375, 1.6],
        [0, -0.75, 1.2],
        [0, -0.6, 1.2],
        [0, -0.4, 1.2],
        [0, -0.25, 1.2],
        [0, -0.9, 0.5],
        [0, -0.1, 0.5]
    ]) 

def S():
    return np.array([
        [0, 0.3, 1.95],   
        [0, -0.05, 2],  
        [0, -0.2, 1.7],  
        [0, 0.0, 1.35],  
        [0, 0.2, 1.1],   
        [0, 0.3, 0.75],   
        [0, 0.0, 0.5], 
        [0, -0.3, 0.5],  
        [0, -0.5, 0.7]   
    ])

def C():
    return np.array([
        [0, 0.4, 1.8],   
        [0, 0.1, 2.0],   
        [0, -0.2, 2.0],  
        [0, -0.4, 1.675],  
        [0, -0.5, 1.25], 
        [0, -0.4, 0.875],  
        [0, -0.2, 0.5],  
        [0, 0.1, 0.5],   
        [0, 0.4, 0.7]   
    ])

def move(swarm,points):
################################## CBF-QP Controller ####################################
    D_MIN = 0.5
    num_drones =2
    num_constraints = int((num_drones-1)*(num_drones)/2)
    u1 = cp.Variable((num_drones*3,1))
    u1_des = cp.Parameter((num_drones*3,1),value = np.zeros((num_drones*3,1)) )
    A1 = cp.Parameter((num_constraints,num_drones*3),value=np.zeros((num_constraints,num_drones*3)))
    b1 = cp.Parameter((num_constraints,1),value=np.zeros((num_constraints,1)))
    const1 = [A1 @ u1 >= b1]
    objective1 = cp.Minimize( cp.sum_squares( u1 - u1_des  ))
    cbf_controller = cp.Problem( objective1, const1 )
    #########################################################################################
    points = points[:num_drones]
    current_pos = np.array([drone.position()[:3] for drone in swarm.allcfs.crazyflies])
    # Solves the central CBF-QP until the drones are close to their initial positions while enforcing collision avoidance
    while np.any(np.linalg.norm(current_pos-points, axis=1) > 0.05):
        for i in range(num_drones):
            vector = (points[i] - current_pos[i]).reshape(-1,1)
            if np.linalg.norm(vector) <= 0.05:
                vector = np.zeros((3,1))
            else:
                vector= vector/np.linalg.norm(vector)/3
            u1_des.value[3*i:3*i+3] = vector
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
        swarm.timeHelper.sleep(0.01)
        current_pos = np.array([drone.position()[:3] for drone in swarm.allcfs.crazyflies])
    # Stops the drones 
    for i in range(num_drones):
        swarm.allcfs.crazyflies[i].cmdVelocityWorld(np.zeros(3,), yawRate=0)

def main():
    swarm = Crazyswarm()
    hover_all(swarm, HEIGHT, Z_SPEED)
    letters = [D,A,S,C]
    for letter in letters:
        points = letter()
        move(swarm, points)


if __name__ == "__main__":
    main()


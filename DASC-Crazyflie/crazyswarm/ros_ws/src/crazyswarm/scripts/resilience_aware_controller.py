import cvxpy as cp
import matplotlib.pyplot as plt
import numpy as np
from pycrazyswarm import *
import sys
import os 
import gurobipy
sys.path.append(os.path.join(os.path.dirname(__file__), 'resilience_aware_'))
from single_integrator import *
from helper import *



# Initialize the robots
def main():

    scenario = 1

    # Sim Parameters  
    F = 2
    R = 4
    d_min = 0.5
    num_constraints1  = 1
    alphas = 0.1
    umax = 1
    tau = 1
    T = 30
    dt =0.005
    max_T = T/dt
    step_size = tau/dt

    Z = 4 #m
    TAKEOFF_SPEED = 1 # m/s
    TAKEOFF_DURATION = Z/TAKEOFF_SPEED 
    MOVEMENT_DURATION = 0.002
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs
    dimension = 3
    n = len(allcfs.crazyflies)

    swarm.allcfs.takeoff(targetHeight=Z, duration=TAKEOFF_DURATION)
    timeHelper.sleep(TAKEOFF_DURATION + 4)

    F_prime = F + n // 2
    #Initialize the robot setup 
    robots = []
    for i in range(n-F):
        robots.append(Agent(F, i))
    for i in range(n-F,n):
        robots.append(Malicious(F,i))
    ############################## Optimization problems ###################################
    u1 = cp.Variable((dimension,1))
    u1_des = cp.Parameter((dimension,1),value = np.zeros((dimension,1)) )
    A1 = cp.Parameter((num_constraints1,dimension),value=np.zeros((num_constraints1,dimension)))
    b1 = cp.Parameter((num_constraints1,1),value=np.zeros((num_constraints1,1)))
    const1 = [A1 @ u1 >= b1]
    const1+= [u1<=umax, -umax<=u1]
    objective1 = cp.Minimize( cp.sum_squares( u1 - u1_des  ))
    cbf_controller = cp.Problem( objective1, const1 )
    ########################################################################################

    # Setting up the goal location
    goal = np.array([[0,1],[3.5,0]])

    # Start the simulation 
    counter = 0
    H =[[] for i in range(n)]
    while True:   
        x = np.array([allcfs.crazyflies[i].position()[0:dimension] for i in range(n)])
        # Compute the actual robustness
        edges = []
        for i in range(n):
            for j in range(i+1,n):
                if np.linalg.norm(x[i]-x[j]) <=R:
                    edges.append((i,j))
        # Agents form a network
        for (i,j) in edges:
            robots[i].connect(robots[j])
            robots[j].connect(robots[i])

        # Get the nominal control input
        u_des = []
        for i in range(n):
            helper = (-1)**i *goal[1]
            if i in range(6):
                helper= (-1)**i * goal[0]
                if (-1)**i <0: 
                    helper[1]-=3
            new = 4
            if i==2 or i==4 or i==6 or i==8:
                new = 20
            helper = np.append(helper, new)
            vector = (helper - x[i]).reshape(-1,1)
            u_des.append(vector/np.linalg.norm(vector)/3) 

        # Compute the h_i and \frac{\partial h_i/} {\partial x}
        h, der_ = compute_h_and_der(n,x,edges,R, dimension)
        h = h-F_prime

        # Store the connectivity levels
        for i in range(n):
            H[i].append(h[i])

        #Set up the constraint of QP
        A1.value[:,:]=0
        b1.value[:,:]=0
        control_input = []

        # Set up the weight w and h_hat values according to the scenario
        w = [21, 35]    
        h_hat = h
        if scenario == 2:
            w = [23, 33]    
            h_hat[-F:]+=2.7
            alphas = 0.1
        elif scenario ==3:
            w = [23, 30]    
            h_hat[-F:]-=2

        if np.any(h_hat<0):
            print(counter, h_hat)

        # Each robot constructs the QP and computes its contorl input u_i locally
        for i in range(n):
            u1_des.value = u_des[i]
            N_i = robots[i].neighbors_id()
            B_i = np.append(N_i,i)
            c = []; c_der_ = []
            for j in N_i:
                h_ij, dh_dxi, _ = agent_barrier(x[i], x[j],d_min)
                c.append(h_ij)
                c_der_.append(dh_dxi)
            c = np.array(c); c_der_ = np.array(c_der_)
            c_exp_list = np.exp(-w[1]*c).reshape((1,-1))
            c_exp_der = c_exp_list*w[1]
            exp_list = np.exp(-w[0]*(h_hat[B_i]-0.2)).reshape((1,-1))
            # exp_list1 = np.exp(-w[0]*(h_hat[i])).reshape((1,-1))

            exp_der = exp_list*w[0]
            A1.value[0,:]= exp_der @ (der_[B_i,i].reshape(-1,dimension)) + c_exp_der @ (c_der_.reshape(-1,dimension))
            b1.value[0,0]= -alphas*(1/n-sum(exp_list[0])/(F_prime+1)) + alphas*(sum(c_exp_list[0]))/2
            # b1.value[0,0]= -alphas*(1/n-exp_list1) + alphas*(sum(c_exp_list[0]))/2

            cbf_controller.solve(solver = "GUROBI")
            if cbf_controller.status!='optimal':
                vec = np.array([0.0]*3)
                control_input.append(vec.reshape((-1,1)))
            else:
                control_input.append(u1.value)

        # Agents share their values with neighbors every tau seconds
        if counter >50 and counter % step_size ==0:
            for aa in robots:
                aa.propagate()
            # The agents perform W-MSR
            for aa in robots:
                aa.w_msr()
            # All the agents update their LED colors based on its consensus states (it is used to color the past trajectories of each agents)
            for aa in robots:
                aa.set_color()
        # Each robot implements its own control input u_i            
        for i in range(n):
            mag = np.linalg.norm(control_input[i])
            if mag > .4:
                control_input[i] = control_input[i]/(mag*3)
            swarm.allcfs.crazyflies[i].cmdVelocityWorld(control_input[i].reshape(-1,3)[0], yawRate=0)
            robots[i].reset_neighbors()
        timeHelper.sleep(MOVEMENT_DURATION)

        counter+=1
        if counter>=max_T:
            break
    swarm.return_initial_controller()
    swarm.land_all()


if __name__ == "__main__":
    main()

import numpy as np
from pycrazyswarm import *
import jax.numpy as jnp
from jax import jacrev, hessian, lax, jit
from Strongly_r_robust.Boot_per_QP_solver import QP
import matplotlib.pyplot as plt
import time
from random import randint
from Strongly_r_robust.Robot_Model import *


n = num_robots = 8

leaders = 4
r= leaders -1
dif = 2.5
q1 = 0.02
p1 = jnp.log(1/q1)
q2 = 0.02
p2 = jnp.log(1/q2)
s_A = 6
s = 3
relu = lambda x: (1+q1)/(1+jnp.exp(-s_A*x+p1))-q1
relu2 = lambda x: (1+q2)/(1+jnp.exp(-s*x+ p2))-q2

@jit 
def barrier_func(x):
    def AA(x):
        A = jnp.array([[0.0 for i in range(n)] for j in range(n)]) 
        for i in range(n):
            for j in range(i+1, n):
                dis = dif-jnp.linalg.norm(x[i]-x[j])
                A = A.at[j,i].set(relu(dis))
                A = A.at[i,j].set(relu(dis))  
        return A
    def body(i, inputs):
        temp_x = A @ jnp.concatenate([jnp.array([1.0 for p in range(leaders)]),inputs])
        state_vector = relu2(temp_x[leaders:]-r)
        return state_vector
    
    state_vector = jnp.array([0.0 for p in range(n-leaders)])
    A = AA(x)
    delta = 4
    x = lax.fori_loop(0, delta, body, state_vector) 
    return x

barrier_grad = jit(jacrev(barrier_func))
barrier_double_grad = jit(hessian(barrier_func))

def smoothened_strongly_r_robust_simul(robots):   
    h = barrier_func(robots)
    h_dot = barrier_grad(robots)
    h_ddot =  barrier_double_grad(robots)
    return h, h_dot, h_ddot


compiled = jit(smoothened_strongly_r_robust_simul)


def main():
    TAKEOFF_DURATION = 2.5
    HOVER_DURATION = 4.0
    Z = 0.6
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    #Initialize the robustness maintenace setup 
    n = len(allcfs.crazyflies)
    F=1
    leaders = 4
    broadcast_value = randint(300,1000)/1000
    robots_list = []
    for i in range(leaders):
        robots_list.append(Leaders(i,broadcast_value,F))
    robots_list.append(Malicious(leaders, [0,1000]))
    for i in range(leaders+1, n):
        robots_list.append(Agents(i,F))

    H = [[] for i in range(n-leaders)]
    R = []
    velocity = np.array([[0.0,0.0] for i in range(n)])
    locations = np.array([allcfs.crazyflies[i].position()[0:2] for i in range(n)])
    _,_, _ = QP(n, leaders, locations, velocity/3, compiled)

    allcfs.takeoff(targetHeight=Z, duration=1.0+Z)
    timeHelper.sleep(2+Z)

    iteration_num=0
    try:
        while iteration_num<1100:
            locations = np.array([allcfs.crazyflies[i].position()[0:2] for i in range(n)])
            if iteration_num!=0 and iteration_num % 10==0:
                for i in range(n):
                    for j in range(i+1,n):
                        if np.linalg.norm(locations[i]-locations[j])<=2.5:
                            robots_list[i].connect(robots_list[j])
                            robots_list[j].connect(robots_list[i])
                for aa in robots_list:
                    aa.propagate()
                for aa in robots_list:
                    aa.w_msr()            
            initial_time= time.time()
            current_r,x, velocity = QP(n, leaders, locations, velocity/1.5, compiled)

            QP_time = time.time() - initial_time
            timeHelper.sleep(0.01)

            print("QP time:", QP_time)
            for i in range(n):
                cf = swarm.allcfs.crazyflies[i]
                r,g,b = robots_list[i].set_color()
                cf.setLEDColor(r,g,b)
                upward_vel = (0.6-cf.position()[2])/10
                cf.cmdVelocityWorld(np.array([velocity[i][0], velocity[i][1], upward_vel]), yawRate=0)
            for i in range(n-leaders):
                H[i].append(float(x[i]))
            R.append(current_r)
            iteration_num+=1
    except KeyboardInterrupt:
        timeHelper.sleep(1.5)
        length_of_consensus = len(robots_list[0].history)*10
        for aa in robots_list:
            print(np.repeat(np.array(aa.history),10))

        print('Stopped!')



    for cf in swarm.allcfs.crazyflies:
        cf.cmdVelocityWorld(np.array([0, 0, -0.3]), yawRate=0)
    timeHelper.sleep(1.5)
    for aa in swarm.allcfs.crazyflies:
        aa.land(targetHeight=0.04, duration=2.5)
    timeHelper.sleep(1.5+Z)

    length_of_consensus = len(robots_list[0].history)*10
    for aa in robots_list:
        # plt.plot(np.arange(length_of_consensus)*0.05, np.repeat(np.array(aa.history),10))
        ed = np.repeat(np.array(aa.history),10)
        np.set_printoptions(threshold=np.inf)
        print(ed)
    plt.show()


if __name__ == "__main__":
    main()

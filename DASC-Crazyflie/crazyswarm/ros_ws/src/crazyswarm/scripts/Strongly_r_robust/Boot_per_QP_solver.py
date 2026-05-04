import cvxpy as cp
import matplotlib.pyplot as plt
import numpy as np
from jax import numpy as jnp
from random import randint


class circle:

    def __init__(self,x,y,radius,ax,id):
        self.location = np.array([x,y]).reshape(-1,1)
        self.radius = radius
        self.id = id
        self.type = 'circle'
        self.x = self.location.reshape(1,-1)[0]
        self.render(ax)

    def render(self,ax):
        circ = plt.Circle((self.location[0],self.location[1]),self.radius,linewidth = 1, edgecolor='k',facecolor='k')
        ax.add_patch(circ)




def unsmoothened_adjacency(dif, A, robots):
    n= len(robots)
    for i in range(n):
        for j in range(i+1, n):
            norm = np.linalg.norm(robots[i]-robots[j])
            if norm <= dif:
                A[i,j] =1
                A[j,i] =1

plt.ion()
fig = plt.figure()
ax = plt.axes(xlim=(-5,5),ylim=(-5,10)) 
ax.set_xlabel("X")
ax.set_ylabel("Y")
# ax.set_aspect(1)

# ################# Make Obatacles ###############################
obstacles = []
radius = 0.6
# spread
# obstacles.append( np.array([0.0,1.5, radius]) ) 
# obstacles.append( np.array([-0.6,-0.6,radius]))

#complex
# obstacles.append(np.array([-0.6,1.5, radius]))
# obstacles.append(np.array([0.3,-0.9, radius]))
# obstacles.append(np.array([0.9,0.9, 0.2]))
# obstacles.append(np.array([-1.5,0.0, 0.3]))


#corridor
y1 = -0.6#-1.0
y2 = 0.9 #1.0
radius = 0.3
x_1 = -1.8
x_increment = 0.3
for i in range(int( 7/x_increment )):
    obstacles.append( np.array([x_1,y1,radius]) ) # x,y,radius, ax, id
    obstacles.append( np.array([x_1,y2,radius]) )
    x_1+=x_increment
obstacles.append(np.array([-1.8,1.2,0.4]))


num_obstacles = len(obstacles)
# ####################################################################

def agent_barrier(agent1,agent2,d_min):
    h =  np.linalg.norm(agent1 - agent2)**2 - d_min**2 
    dh_dxi = 2*( agent1 - agent2)
    dh_dxj = -2*( agent1 - agent2)
    ddh_xi = 2*np.ones((1,2))
    dxi = dh_dxi.reshape(1,-1)[0]
    dxj = dh_dxj.reshape(1,-1)[0]
    return h, dxi, dxj, ddh_xi



def strongly_r_robust(A,leaders, delta):
    n= len(A)
    max_r = int(n/2) 
    ans_r =0
    if n % 2==1:
        max_r+=1
    for r in range(1,max_r+1):
        x = np.array([1 for p in range(leaders)] + [0 for p in range(n-leaders)])
        for i in range(delta):
            temp_x = A @ x
            temp_x = np.array([np.heaviside(temp_x[k]-r,1) for k in range(n)])
            x = x + temp_x
            x= [np.heaviside(x[k]-1,1) for k in range(n)]
        if (x>=np.array([1 for i in range(n)])).all():
            ans_r=r

                
    print("real_r_robustness:",ans_r)
    return ans_r

def QP(n, leaders, locations, robots_velocity, compiled):
    dif =2.5
    num_robots =n
    ############################## Optimization problems ######################################

    # ###### 1: CBF Controller
    u1 = cp.Variable((2*n,1))
    u1_ref = cp.Parameter((2*n,1),value = np.zeros((2*n,1)) )
    num_constraints1  = 1
    A1 = cp.Parameter((num_constraints1,2*n),value=np.zeros((num_constraints1,2*n)))
    b1 = cp.Parameter((num_constraints1,1),value=np.zeros((num_constraints1,1)))
    const1 = [A1 @ u1 >= b1]
    objective1 = cp.Minimize( cp.sum_squares( u1 - u1_ref  ) )
    cbf_controller = cp.Problem( objective1, const1 )
    ###################################################################################################

    #Setting the goal
    goal = []

    # goal.append(np.array([100.0, 0.0]).reshape(2,-1))
    # goal.append(np.array([0.0, -100.0]).reshape(2,-1))
    # goal.append(np.array([-100.0, 0.0]).reshape(2,-1))
    # goal.append(np.array([0.0, 100.0]).reshape(2,-1))

    goal.append(np.array([3.0, -0.3]).reshape(2,-1))
    goal.append(np.array([3.0, 0.3]).reshape(2,-1))
    goal.append(np.array([3.0, 0.9]).reshape(2,-1))



    inter_collision = int(n*(n-1)/2)
    weight = np.array([6]*(num_robots-leaders) + [10]*inter_collision + [12]*(num_obstacles*n))


    robots_location = np.array(locations)

    A = np.array([[0.0 for i in range(n)] for j in range(n)]) 
    unsmoothened_adjacency(dif, A, robots_location)
    delta = np.sum(A>=0)
    current_r = strongly_r_robust(A,leaders, delta)
    for i in range(n):
        # vector = goal[i % leaders] - robots_location[i].reshape(2,-1) 
        vector = goal[i % (leaders-1)] - robots_location[i].reshape(2,-1) 
        vector = vector/np.linalg.norm(vector)
        current = robots_velocity[i].reshape(2,-1) 
        temp = jnp.array([[vector[0][0]-current[0]], [vector[1][0]-current[1]]])
        u1_ref.value[2*i] = temp[0][0]
        u1_ref.value[2*i+1] = temp[1][0]

    # The connectivity constraint
    x, der_, double_der_  = compiled(robots_location)
    x=np.asarray(x);der_=np.asarray(der_);double_der_=np.asarray(double_der_)
    print(x)
 

    A1.value[0,:] = [0 for i in range(2*num_robots)]
    b1.value[0] = 0

    #Obstacle Collision avoidance and Inter-agent collision avoidance
    collision = [];ob=[]
    col_alpha = 1.5; obs_alpha = 1.5
    for i in range(num_robots):
        for j in range(i+1, num_robots):
            h, dh_dxi, dh_dxj, ddh = agent_barrier(robots_location[i],robots_location[j], 0.3)
            h_dot = dh_dxi @ robots_velocity[i] + dh_dxj @ robots_velocity[j] + col_alpha*h
            collision.append(h_dot)
            kk = num_robots-leaders+j
            if h_dot < 0:
                print("inter", h_dot)
                print(i,j)
            temp = (weight[kk])*np.exp(-weight[kk]*h_dot)
            A1.value[0,2*i:2*i+2]+= temp * dh_dxi[:]
            A1.value[0,2*j:2*j+2]+= temp *dh_dxj[:]
            b1.value[0]-=  temp *(2* robots_velocity[i].reshape(1,-1)[0] @  robots_velocity[i] - 2* robots_velocity[i].reshape(1,-1)[0] @  robots_velocity[j] + col_alpha*dh_dxi @ robots_velocity[i])
            b1.value[0]-=  temp *(2* robots_velocity[j].reshape(1,-1)[0] @  robots_velocity[j] - 2* robots_velocity[i].reshape(1,-1)[0] @  robots_velocity[j] + col_alpha*dh_dxj @ robots_velocity[j])
        for j in range(num_obstacles):
            h, dh_dxi, dh_dxj, ddh = agent_barrier(robots_location[i], obstacles[j][:2],obstacles[j][-1]+0.1)
            if j==3:
                h, dh_dxi, dh_dxj, ddh = agent_barrier(robots_location[i], obstacles[j][:2],obstacles[j][-1]+0.1)
            h_dot =  dh_dxi @ robots_velocity[i] + obs_alpha*h
            ob.append(h_dot)
            kk = num_robots-leaders+inter_collision+i
            temp = (weight[kk])*np.exp(-weight[kk]*h_dot)
            A1.value[0,2*i:2*i+2]+= temp * dh_dxi[:]
            b1.value[0]-= temp *(2* robots_velocity[i].reshape(1,-1)[0] @  robots_velocity[i] + obs_alpha*dh_dxi @ robots_velocity[i])
    
    #Robustness HOCBF 
    alphas = 2
    robustes = []
    for k in range(num_robots-leaders):
        h_dot = der_[k].reshape(1,-1)[0] @ robots_velocity.reshape(-1,1)
        weightee = ((weight[k])*np.exp(-weight[k]*(h_dot+alphas*x[k])))[0]
        robustes.append((h_dot+alphas*x[k])[0])
        if h_dot+alphas*x[k] <0:
            print("robustness", h_dot+alphas*x[k])
        A1.value[0,:]+= weightee * der_[k].reshape(1,-1)[0]

        temp = []
        for j in range(num_robots):
            temp_x = robots_velocity.reshape(1,-1) @ double_der_[k][j][:][:][0].reshape(-1,1)
            temp_y = robots_velocity.reshape(1,-1) @ double_der_[k][j][:][:][1].reshape(-1,1)
            temp.append(temp_x[0])
            temp.append(temp_y[0]) 
        b1.value[0]-= weightee * ((np.array(temp).reshape(1,-1) + alphas*der_[k].reshape(1,-1)) @ robots_velocity.reshape(-1,1))[0]

    #Composition 
    sum_h = 1 - np.sum(np.exp(-weight*np.array(robustes + collision + ob)))
    b1.value[0]-=1.5*(sum_h)
    #Solve the CBF-QP and get the control input \mathbf u
    cbf_controller.solve()

    for i in range(num_robots):
        robots_velocity[i]+=np.array(u1.value[2*i:2*i+2]).reshape(1,-1)[0]*0.05

    return current_r, x, robots_velocity

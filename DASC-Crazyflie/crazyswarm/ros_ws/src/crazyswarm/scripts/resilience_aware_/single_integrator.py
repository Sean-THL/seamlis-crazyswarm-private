import numpy as np
from random import randint
from scipy.integrate import solve_ivp 
import matplotlib.cm as cm


# Computes the distance function h_{ij}^{col} between the agent i and j, and also computes its derivatives w.r.t. both agents
def agent_barrier(agent1, agent2, d_min):
    h =  np.linalg.norm(agent1 - agent2)**2 - d_min**2 
    dh_dxi = 2*( agent1 - agent2).T
    dh_dxj = -2*( agent1 - agent2 ).T
    return h, dh_dxi, dh_dxj


## Normal agents
class Agent:
    def __init__(self, F, id):
        self.value= randint(-500,500)        
        self.original = self.value
        self.connections = []
        self.F = F
        self.values = []
        self.history = [self.value]
        self.id = id
        self.set_color()

    # Forms an edge with the given agent.
    def connect(self, agent):
            self.connections.append(agent)

    # Returns its neighbor set
    def neighbors(self):
        return self.connections

    # Returns the list of neighboring robots
    def neighbors_id(self):
        return [aa.id for aa in self.connections]
    
    # Resets its neighbor set 
    def reset_neighbors(self):
        self.connections = []

    # Sets the agent's LED color based on its consensus state (It is used to color the past trajectory of the agent)
    def set_color(self):
        self.LED = cm.tab20((self.value+500)/1000)

    # Shares its consensus value with its neighbors  
    def propagate(self):
        for neigh in self.neighbors():
            neigh.receive(self.value)
        return self.value

    # Receives the consensus states from its neighbor
    def receive(self, value):
        self.values.append(value)
    
    # Performs W-MSR
    def w_msr(self):
        small_list=[];big_list=[];comb_list=[]

        for aa in self.values:
            if aa<self.value:
                small_list.append(aa)
            elif aa>self.value:
                big_list.append(aa)
            else:
                comb_list.append(aa)

        small_list = sorted(small_list)
        small_list = small_list[self.F:]

        big_list = sorted(big_list)
        big_list = big_list[:-self.F]
        comb_list = small_list+ comb_list + big_list
        total_list =len(comb_list)
        weight = 1/(total_list+1)
        weights = [weight for i in range(total_list)]
        avg = weight*self.value + sum([comb_list[i]*weights[i] for i in range(total_list)])
        self.value = avg

        self.history.append(self.value)
        self.values = []


## Malicious agents
class Malicious(Agent):
    def __init__(self,  F, id):
        super().__init__(F, id)
        self.time = 0
        self.value = 500*np.sin(self.id+self.time/3.5)
        self.history = [self.value]

    # Sends wrong values to all of its neighbors
    def propagate(self):
        self.time+=1
        self.value = 500*np.sin(self.id+self.time/3.5)
        for neigh in self.neighbors():
            neigh.receive(self.value)
        return self.value
    
    # Does not follow W-MSR to update its consensus state
    def w_msr(self):
        self.history.append(self.value)
        self.values = []

## Normal Leaders
class Leaders(Agent):
    def __init__(self, value, F, id):
        super().__init__(F, id)
        self.value=value
        self.history = []

    def propagate(self):
        for neigh in self.neighbors():
            neigh.receive(self.value)
        return self.value
    def receive(self, value):
        pass
    def w_msr(self):
        self.history.append(self.value)
        self.values = []
        self.connections =[]


# class Vector_Leaders(Leaders):
#     def __init__(self, value, location, color, palpha, ax, F, dim):
#         super().__init__(value, location, color, palpha, ax, F)
#         self.dim = dim
#         self.value=value
#         self.history = [[] for i in range(self.dim)]

#     def propagate(self):
#         for neigh in self.neighbors():
#             neigh.receive(self.value)
#         return self.value
#     def receive(self, value):
#         pass
#     def w_msr(self):
#         for i in range(self.dim):
#             self.history[i].append(self.value[i])
#         self.values = []
#         self.connections =[]
#     def update_state(self,value):
#         self.value = value


# class Vector_Followers(Agent):
#     def __init__(self, location, color, palpha, ax, F,dim):
#         super().__init__(location, color, palpha, ax, F)
#         self.dim= dim
#         self.history = [[] for i in range(self.dim)]
#         self.value = []
#         self.values = [[] for i in range(self.dim)]
#     def propagate(self):
#         if len(self.value)!=0:
#             for neigh in self.neighbors():
#                 neigh.receive(self.value)
#         return self.value
    
#     def receive(self, value):
#         for i in range(self.dim):
#             self.values[i].append(value[i])
#     def w_msr(self):
#         self.value = []
#         if len(self.values[0])>=2*self.F+1:
#             for i in range(self.dim):
#                 med = median(self.values[i])
#                 self.value.append(med)
#                 self.history[i].append(med)
#         else:
#             for i in range(self.dim):
#                 self.history[i].append(0)
#         self.connections =[]
#         self.values = [[] for i in range(self.dim)]

    
# class Malicious(Leaders):
#     def __init__(self, range, location, color, palpha, ax, F):
#         self.range = range
#         self.value = randint(range[0], range[1])
#         super().__init__(self.value, location, color, palpha, ax, F)
#     def propagate(self):
#         self.value = randint(self.range[0], self.range[1])
#         for neigh in self.neighbors():
#             neigh.receive(self.value)
#         return self.value
    

# class Vector_Malicious(Vector_Followers):
#     def __init__(self, rangee, location, color, palpha, ax, F,dim):
#         super().__init__ (location, color, palpha, ax, F,dim)
#         self.range = rangee
#         self.dim = dim
#         self.value = [randint(rangee[0], rangee[1])for i in range(self.dim)]

#     def propagate(self):
#         self.value = [randint(self.range[0], self.range[1])for i in range(self.dim)]
#         for neigh in self.neighbors():
#             neigh.receive(self.value)
#         return self.value
#     def receive(self, value):
#         pass
#     def w_msr(self):
#         self.values = []
#         self.connections =[]
#         for i in range(self.dim):
#             self.history[i].append(self.value[i])
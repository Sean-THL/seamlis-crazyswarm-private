import numpy as np
from random import randint

class Agent:
    def __init__(self,location, color, palpha, ax, F):
        self.location = location.reshape(4,-1)
        self.locations = []
        self.Us = []
        self.color = color
        self.palpha = palpha
        self.body = ax.scatter([],[],c=color,alpha=palpha,s=10)
        self.obs_h = np.ones((1,2))
        self.obs_alpha =  2.0*np.ones((1,2))#
        self.value= randint(-10,10)
        self.original = self.value
        self.connections = []
        self.F = F
        self.x = self.location.reshape(1,-1)[0][0:2]
        self.values = []
        self.history = []

    def f(self):
        return np.array([[0,0,1,0],[0,0,0,1],[0,0,0,0],[0,0,0,0]])
    
    def g(self):
        return np.array([ [0,0],[0,0],[1, 0],[0, 1] ])
    
    def step(self,U): #Just holonomic X,T acceleration
        self.U = U.reshape(2,1)
        self.location = self.location + (self.f() @ self.location + self.g() @ self.U )*0.03
        self.x = self.location.reshape(1,-1)[0][0:2]
        self.render_plot()
        temp = np.array([self.U[0][0],self.U[1][0]])
        self.Us = np.append(self.Us,temp)
        return self.location

    def render_plot(self):
        # scatter plot update
        self.locations = np.append(self.locations, self.x)
        self.body.set_offsets(self.x)
        #animate(x)

    def agent_barrier(self,agent,d_min):
        h =  np.linalg.norm(self.x - agent.x)**2 - d_min**2 
        dh_dxi = 2*( self.x - agent.x)
        dh_dxj = -2*( self.x - agent.x)
        ddh_xi = 2*np.ones((1,2))
        dxi = np.array([dh_dxi, ddh_xi[0]]).reshape(1,-1)[0]
        dxj = np.array([dh_dxj, ddh_xi[0]]).reshape(1,-1)[0]
        return h, dxi, dxj

    def connect(self, agent):
            self.connections.append(agent)

    def neighbors(self):
        return self.connections

    def propagate(self):
        if self.value!=self.original:
            for neigh in self.neighbors():
                neigh.receive(self.value)
        return self.value

    def receive(self, value):
        self.values.append(value)
    
    def w_msr(self):
        small_list=[];big_list=[];comb_list=[]
        if len(self.values)>=2*self.F+1:
            for aa in self.values:
                if aa<self.value:
                    small_list.append(aa)
                elif aa>self.value:
                    big_list.append(aa)
                else:
                    comb_list.append(aa)

            if len(small_list) <=self.F:
                small_list = []
            else:
                small_list = sorted(small_list)
                small_list = small_list[self.F:]

            if len(big_list) <=self.F:
                big_list = []
            else:
                big_list = sorted(big_list)
                big_list = big_list[:len(big_list)-self.F]

            comb_list = small_list+ comb_list + big_list
            total_list =len(comb_list)
            weight = 1/(total_list+1)
            weights = [weight for i in range(total_list)]
            avg = weight*self.value + sum([comb_list[i]*weights[i] for i in range(total_list)])

            self.value = avg
        self.history.append(self.value)
        self.values = []
        self.connections =[]


class Leaders(Agent):
    def __init__(self, value, location, color, palpha, ax, F):
        super().__init__(location, color, palpha, ax, F)
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
    
class Malicious(Leaders):
    def __init__(self, range, location, color, palpha, ax, F):
        self.range = range
        value = randint(range[0], range[1])
        super().__init__(value, location, color, palpha, ax, F)
    def propagate(self):
        self.value = randint(self.range[0], self.range[1])
        for neigh in self.neighbors():
            neigh.receive(self.value)
        return self.value
    


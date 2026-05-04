import random
import numpy as np
import matplotlib.pyplot as plt
from statistics import median

# Agent whose w_msr is for scalar value consensus
class Agents:
    def __init__(self,id, value, F):
        self.id = id
        self.value =value
        self.F = F
        self.values = []
        self.history =[]
        self.connections = []
    
    def connect(self, agents):
        for aa in agents:
            self.connections.append(aa)

    def neighbors(self):
        return self.connections

    def propagate(self):
        for neigh in self.neighbors():
            neigh.receive(self.value)
        return self.value

    def receive(self, value):
        self.values.append(value)
    
    def w_msr(self):
        small_list=[];big_list=[];comb_list=[]
        self.history.append(self.value)
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
        self.values = []


class Leaders(Agents):
    def __init__(self,id, value, F):
        self.value = value
        super().__init__(id, value, 0)
        self.history = [[]for i in range(len(self.value))]
        self.values = [[]for i in range(len(self.value))]

    def propagate(self):
        for neigh in self.neighbors():
            neigh.receive(self.value)
        return self.value
    def receive(self, value):
        for i in range(len(self.value)):
            self.values[i].append(value[i])
    def w_msr(self):
        self.values = [[]for i in range(len(self.value))]
        for i in range(len(self.value)):
            self.history[i].append(self.value[i])



class Followers(Agents):
    def __init__(self,id, value, F, dim):
        super().__init__(id, 0, F)
        self.value = [[] for i in range(dim)]
        self.history = [[]for i in range(dim)]
        self.values=[[]for i in range(dim)]
        self.dim =dim
    def propagate(self):
        if self.value!=[[] for i in range(self.dim)]:
            for neigh in self.neighbors():
                neigh.receive(self.value)
        return self.value
    def receive(self, value):
        for i in range(len(self.value)):
            self.values[i].append(value[i])
    def w_msr(self):
        if self.values!=[[] for i in range(self.dim)]:
            for i in range(len(self.value)):
                self.value[i] = median(self.values[i])
        for i in range(len(self.value)):
            self.history[i].append(self.value[i])
        self.values=[[]for i in range(len(self.value))]




class Adversary:
    def __init__(self,id, value):
        self.id = id
        self.value = value
        self.values = []
        self.history =[]
        self.connections = []
    
    def connect(self, agents):
        for aa in agents:
            self.connections.append(aa)

    def neighbors(self):
        return self.connections

    def propagate(self):
        for neigh in self.neighbors():
            neigh.receive(self.value)
        return self.value

    def receive(self, value):
        self.values.append(value)
        return self.value
    
    def w_msr(self):
        self.history.append(self.value)
        self.values = []


class Byzantine(Adversary):
    def __init__(self, id, range, dim=1, time =0):
        super().__init__(id, 0)
        self.low, self.high = range
        if self.low>self.high:
            self.low, self.high = self.high, self.low
        self.dim = dim
        self.time = time
    def propagate(self):
        neigh = self.neighbors(); num = len(neigh)

        if num ==0:
            num =5
            temp_list = [200*(-1)**i*(np.sin(self.time+5*self.id+3)) for i in range(num)]
            self.time+=1
            self.value = sum(temp_list)/num
            num=0
        
        if num!=0:
            temp_list = [np.array([random.randint(-2,2) + 5*np.sin(self.time+self.id)  for j in range(self.dim)]) for i in range(num)]
            #temp_list = [50*(-1)**self.id*(np.sin(self.time)) for i in range(num)]
            self.time+=1
            self.value = sum(temp_list)/num
            for k in range(num):
                neigh[k].receive(temp_list[k])
 
        return self.value
    
class Malicious(Adversary):
    def __init__(self, id, range, dim =1):
        super().__init__(id, 0)
        self.time = 0
        self.low, self.high = range
        if self.low>self.high:
            self.low, self.high = self.high, self.low
        self.dim = dim
    def propagate(self):
        self.value = np.array([1000*np.sin(self.time+self.id) for i in range(self.dim)])
        #self.value = np.array([100*self.time for i in range(self.dim)])
        self.time+=1
        for neigh in self.neighbors():
            neigh.receive(self.value)
        return self.value
    
class MaliciousFollower(Adversary):
    def __init__(self, id, range, F, dim=1):
        super().__init__(id, 0)
        self.low, self.high = range
        if self.low>self.high:
            self.low, self.high = self.high, self.low
        self.dim = dim
        self.F = F
    def propgate(self):
        if len(self.values)>=2*self.F+1:
            self.value = np.array([random.randint(self.low,self.high) for i in range(self.dim)])
            for neigh in self.neighbors():
                neigh.receive(self.value)
        return self.value
    
class VectorMalicious(Adversary):
    def __init__(self, id, rangee, dim =1):
        super().__init__(id, 0)
        self.low, self.high = rangee
        if self.low>self.high:
            self.low, self.high = self.high, self.low
        self.dim = dim
        self.history = [[] for i in range(dim)]
        self.values = [[] for i in range(dim)]
        self.value = [[] for i in range(dim)]

    def propagate(self):
        for i in range(self.dim):
            self.value[i] = random.randint(self.low,self.high)
        for neigh in self.neighbors():
                neigh.receive(self.value)
        return self.value
    
    def w_msr(self):
        for i in range(self.dim):
            self.history[i].append(self.value[i])
        self.values = [[]for i in range(self.dim)]
        self.value = [[] for i in range(self.dim)]






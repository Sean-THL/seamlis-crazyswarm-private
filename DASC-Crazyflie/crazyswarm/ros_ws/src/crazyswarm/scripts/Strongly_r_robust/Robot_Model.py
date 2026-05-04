import random
import numpy as np
import matplotlib.pyplot as plt
from statistics import median
from random import randint
import colorsys
import matplotlib.colors as mcolors
# Agent whose w_msr is for scalar value consensus
class Agents:
    def __init__(self,id, F):
        self.id = id
        self.value= randint(0,400)/1000
        self.F = F
        self.values = []
        self.history =[self.value]
        self.connections = []
        self.color = None
    
    def connect(self, agents):
        self.connections.append(agents)

    def neighbors(self):
        return self.connections

    def propagate(self):
        for neigh in self.neighbors():
            neigh.receive(self.value)
        return self.value

    def receive(self, value):
        self.values.append(value)

    def set_color(self, colormap='hsv'):
        cmap = plt.get_cmap(colormap)
        rgba = cmap(self.value)
        rgb = [rgba[0], rgba[1], rgba[2]]
        return rgb
        # return [int(256* c) for c in rgb]
    
    def w_msr(self):
        small_list=[];big_list=[];comb_list=[]
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
        self.set_color()
        self.history.append(self.value)
        self.values = []


class Leaders(Agents):
    def __init__(self,id, value, F):
        super().__init__(id, 0)
        self.value = value

        self.history = [self.value]
        self.values = []

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
    def __init__(self, id, range):
        self.range = range
        value = (randint(range[0], range[1]))/1000
        super().__init__(id, value,1)
    def propagate(self):
        self.value = randint(self.range[0], self.range[1])/1000
        for neigh in self.neighbors():
            neigh.receive(self.value)
        return self.value


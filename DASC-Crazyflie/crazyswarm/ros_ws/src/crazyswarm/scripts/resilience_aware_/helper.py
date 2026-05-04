import numpy as np
import matplotlib.pyplot as plt
from single_integrator import *

# Allows to choose which scenario to simulate among three scenarios
# def choose():
#     scenario = input("Please press the number:\n1:nominal update\n2:overstatement \n3:understatement\n")
#     while True:
#         try:
#             scenario = int(scenario)
#             if 0<scenario<4:
#                 match scenario:
#                     case 1:
#                         print("Simulating nominal update scenario...")
#                     case 2:
#                         print("Simulating overstatement scenario...")
#                     case 3:
#                         print("Simulating understatement scenario...")       
#                 return scenario
#             else:
#                 scenario = input("Wrong input, please press the number:\n1:nominal update\n2:overstatement \n3:understatement\n")
#         except:
#             scenario = input("Wrong input, please press the number:\n1:nominal update\n2:overstatement \n3:understatement\n")


# Compute the h_i and \frac{\partial h_i/} {\partial x}
def compute_h_and_der(n,x,edges,R, dimension):

    #Define the parametrized sigmoid function and its derivative
    eps = 1/(n-1)- 0.001
    q1 = 2+eps
    q2 = 0.9 
    sigmoid_A1 = lambda x: q1 / (1+np.exp(-q2*x)) - q1/2
    def compute_der(x_i, x_j):

        dist = (R**2 - np.sum((x_i-x_j)**2))**2
        exp_term = 2 * q1 * q2* np.exp(-q2 *dist)
        denominator = (1 + exp_term)**2
        coefficient = exp_term / denominator

        return - coefficient * ((x_i - x_j))*2*(R**2 - np.sum((x_i-x_j)**2))
    
    # Compute h and \frac{\partial h_i/} {\partial x} 
    h = np.zeros(n)
    der_ = np.zeros((n,n,dimension))
    for (i,j) in edges:
        dist = R**2-np.sum((x[i]-x[j])**2)
        h[i]+= sigmoid_A1(dist)
        h[j]+= sigmoid_A1(dist)
        der_[i,i]+=compute_der(x[i],x[j])
        der_[i,j]= -compute_der(x[i],x[j])
        der_[j,j]+=compute_der(x[j],x[i])
        der_[j,i]= -compute_der(x[j],x[i]) 
    return h, der_


# Save the animations of robots' h_i and y_i values with respect to t
def generate_anim(robots, counter, dt, H, step_size):
    from matplotlib.animation import FuncAnimation, FFMpegWriter
    for aa in robots:
        aa.history = np.repeat(np.array(aa.history), step_size)

    x_axis = np.arange(counter) * dt
    length_of_consensus = len(x_axis)
    fig1, ax1 = plt.subplots()
    fig2, ax2 = plt.subplots()

    def animation(p):
        ax1.clear()
        ax1.set_xlim(0, length_of_consensus*dt)
        for i in range(n):
            if issubclass(type(robots[i]), Malicious):
                ax1.plot(x_axis[:p], H[i][:p], "r--")
            else:
                ax1.plot(x_axis[:p], H[i][:p], color=f"C{i % 10}")
        ax1.plot(x_axis, [0]*length_of_consensus, 'k--')

    anim1 = FuncAnimation(fig1, func=animation, frames=np.arange(0, length_of_consensus), interval = 100)
    writer1 = FFMpegWriter(fps=200)
    anim1.save('connectivity_understatement.mp4', writer=writer1)

    def animation2(p):
        ax2.clear()
        ax2.set_xlim(0, length_of_consensus*dt)
        ax2.set_ylim(-500, 500)

        for idx, aa in enumerate(robots):
            if issubclass(type(aa), Malicious):
                ax2.plot(x_axis[:p], aa.history[:p], "r--")
            else:
                ax2.plot(x_axis[:p], aa.history[:p], color=f"C{idx % 10}")

    anim2 = FuncAnimation(fig2, func=animation2, frames=np.arange(0, len(robots[0].history)))
    writer2 = FFMpegWriter(fps=200)
    anim2.save('consensus_understatement.mp4', writer=writer2)
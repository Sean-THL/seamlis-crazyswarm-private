import numpy as np

def adjacency_undirected(edges,A):
    for (i,j) in edges:
        A[i][j]=1
        A[j][i]=1

def sigmoid(value,v):
    return 1/(1+np.exp(-5.5*(value-v)))


def smoothened_adjacency(dif, A, robots):
    sigmit = lambda x: np.exp(-10*(np.sqrt(((x[0]-x[2])**2+(x[1]-x[3])**2))-x[4]))/(1+np.exp(-10*(np.sqrt(((x[0]-x[2])**2+(x[1]-x[3])**2))-x[4])))
    n = len(A)
    for i in range(n):
        for j in range(i+1, n):
            A[j][i] = sigmit([robots[j][0], robots[j][1], robots[i][0],robots[i][1], dif-0.5])
            A[i][j] = sigmit([robots[i][0], robots[i][1], robots[j][0],robots[j][1], dif-0.5]) 

def unsmoothened_adjacency(dif, A, robots):
    n = len(A)
    for i in range(n):
        for j in range(i+1, n):
            norm = np.linalg.norm(robots[i]-robots[j])
            if norm <= dif:
                A[i][j] =1
                A[j][i] = 1

def rdeg(n,r):
    groups = int(n/r)
    edges=[]
    for i in range(groups-1):
        for j in range(r):
            for z in range(r):
                edges.append((r*i+j, r*(i+1)+z))
    for i in range(r*(groups-1), r*(groups)):
        for j in range(r*groups, n):
            edges.append((i,j))
    return edges




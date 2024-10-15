import gurobipy as gp
from gurobipy import GRB
import numpy as np

# Matrix
# Ay>=d
# A is the matrix to fullfill zi < 800yi
A = np.array([[800,   0,   0,  -1.,   0,   0],
            [  0, 800,   0,   0,  -1,   0],
            [  0,   0, 800,   0,   0,  -1],
            [  0,   0,   0,   1,   1,   1]])
d = np.array([0,0,0,772])
# Ey+Gx>h-Mu

# G is a flow where row is customer and facility, column is arc from facility to customer\
G = np.array([[-1, -1, -1,  0,  0,  0,  0,  0,  0],
            [ 0,  0,  0, -1, -1, -1,  0,  0,  0],
            [ 0,  0,  0,  0,  0,  0, -1, -1, -1],
            [ 1,  0,  0,  1,  0,  0,  1,  0,  0],
            [ 0,  1,  0,  0,  1,  0,  0,  1,  0],
            [ 0,  0,  1,  0,  0,  1,  0,  0,  1]])
# E is the matrix to fullfill sum xij = zi for all i
E = np.array([[  1,   0,   0],
              [  0,   1,   0],
              [  0,   0,   1],
              [  0,   0,   0],
              [  0,   0,   0],
              [  0,   0,   0]])
# M is for demand deviation
M = np.array([[  0,   0,   0],
            [  0,   0,   0],
            [  0,   0,   0],
            [-40,   0,   0],
            [  0, -40,   0],
            [  0,   0, -40]])


# h is the base demand
h = np.array([  0,   0,   0, 206, 274, 220])

# cost vectors
f = np.array([400, 414, 326])
a = np.array([18, 25, 20])
c = np.array([22, 33, 24, 33, 23, 30, 20, 25, 27])
# bounds
LB = -GRB.INFINITY
UB = GRB.INFINITY
# tolerance
tolerance = 1e-4
iteration = 0


# define master problem
master = gp.Model("master_problem")
master.Params.LogToConsole = 0
y = master.addMVar((3,), vtype=GRB.BINARY)
z = master.addMVar((3,), lb=0,vtype=GRB.CONTINUOUS)
eta = master.addVar(lb=0, vtype=GRB.CONTINUOUS)
for i in range(A.shape[0]):
    master.addConstr(
        gp.quicksum(A[i, j] * y[j] for j in range(3)) +  # Part for y
        gp.quicksum(A[i, j+3] * z[j] for j in range(3))  # Part for z
        >= d[i]
    )
master.setObjective(f@y+a@z+eta,GRB.MINIMIZE)


while abs(UB-LB) >= tolerance:
    iteration = iteration + 1
    print("Iteration :",iteration)
    master.optimize()
    if master.status == GRB.OPTIMAL:
        LB = max(LB,master.ObjVal)
    sub = gp.Model("sub_problem")
    sub.Params.LogToConsole = 0
    x = sub.addMVar((9,), lb=0,vtype=GRB.CONTINUOUS)
    g = sub.addMVar((3,),lb=0,ub=1, vtype=GRB.CONTINUOUS)
    pi = sub.addMVar(6, lb=0, vtype=GRB.CONTINUOUS, name="pi")  # Dual variables
    # sub constraints
    sub.addConstr(G@x >= h-E@z.X-M@g)
    # Dual constraint: G.T @ pi <= b
    sub.addConstr(np.transpose(G)@pi <= np.transpose(c))
    
    for i in range(6):
        sub.addConstr((G@x - h + E@z.X-M@g)[i] * pi[i] == 0)

    for i in range(9):
        sub.addConstr((np.transpose(c)-np.transpose(G)@pi)[i]*x[i] ==0)

    sub.addConstr(g[0] + g[1] + g[2] <= 1.8)
    sub.addConstr(g[0] + g[1] <= 1.2)
    sub.setObjective(c@x,GRB.MAXIMIZE)
    sub.optimize()
    if sub.status == GRB.OPTIMAL:
        L_val = sub.ObjVal
    else:
         print("no sub solution")
         break
    UB= min(UB,f@y.X+a@z.X+L_val)
    print("UB =",UB)
    print("LB =",LB)
    x_new = x.X
    if L_val < GRB.INFINITY:
           master.addConstr(eta>=L_val)
    master.addConstr(G@x_new+E@z >= h-M@g.X)
    print("x",x.X)
    print("g",g.X)

print("final output")
print(z.X)
print(y.X)
print("x",x.X)
print("g",g.X)
print(G@x.X)
print(E@z.X)
print(h-E@z.X-M@g.X)


           

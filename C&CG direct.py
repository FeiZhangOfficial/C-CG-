import gurobipy as gp
from gurobipy import GRB, quicksum
import numpy as np
# Create sub


# Define variables

f = np.array([400, 414, 326])
a = np.array([18, 25, 20])
c = np.array([[22, 33, 24],
            [33, 23, 30],
            [20, 25, 27]])

I = 3
J = 3
master = gp.Model("master_problem")
master.Params.LogToConsole = 0
y = master.addMVar((I,), vtype=GRB.BINARY)
z = master.addMVar((I,), lb=0, vtype=GRB.CONTINUOUS)

eta = master.addVar(lb=0, vtype=GRB.CONTINUOUS)
master.addConstr(z<=800*y)
master.addConstr(quicksum(z[i] for i in range(I))>=772)
master.setObjective(f@y + a@z + eta, GRB.MINIMIZE)
master.optimize()
LB = -GRB.INFINITY
UB = GRB.INFINITY
tolerance = 1e-4
iteration = 0
if master.status == GRB.OPTIMAL:
    LB = max(LB, master.ObjVal)
z_star = z.X
h = np.array([206, 274, 220])

sub = gp.Model("sub_problem")
sub.Params.LogToConsole = 0
alpha = sub.addMVar((I, J), vtype=GRB.BINARY, name="alpha")
beta = sub.addMVar(J, vtype=GRB.BINARY, name="beta")
gamma = sub.addMVar(I, vtype=GRB.BINARY, name="gamma")
x = sub.addMVar((I, J),lb=0,vtype=GRB.CONTINUOUS, name="x")
g = sub.addMVar(I, lb = 0, ub = 1,vtype=GRB.CONTINUOUS, name="g")
pi = sub.addMVar(I, vtype=GRB.CONTINUOUS, name="pi")
lambda_var = sub.addMVar(J, vtype=GRB.CONTINUOUS, name="lambda")
d = sub.addMVar(J,vtype=GRB.CONTINUOUS)
sub.addConstr(d == h+g*40)
bigm = 1e3
print(z_star)
for i in range(I):
    sub.addConstr(gp.quicksum(x[i, j] for j in range(J)) <= z_star[i],name="c1")
    sub.addConstr(z_star[i]-quicksum(x[i, j] for j in range(J)) <=(1-gamma[i]*z_star[i]),name = "c2")
    sub.addConstr(pi[i]<=gamma[i]*bigm)
for j in range(J):
    sub.addConstr(quicksum(x[i, j] for i in range(I)) >= d[j])
    sub.addConstr(lambda_var[j]<=bigm*beta[j])
    sub.addConstr(quicksum(x[i, j] for i in range(I))-d[j] <= g[j]*(1-beta[j]))
for i in range(I):
    for j in range(J):
        sub.addConstr(lambda_var[j] - pi[i] <= c[i,j])
        sub.addConstr(x[i,j]<=bigm * alpha[i,j])
        sub.addConstr(c[i,j]-lambda_var[j]+pi[i]<=(c[i][j]+bigm) * (1-alpha[i,j]))
sub.addConstr(g[0]+g[1]+g[2]<=1.8)
sub.addConstr(g[0]+g[1]<=1.2)
sub.setObjective(quicksum(c[i,j] * x[i,j] for i in range(I) for j in range(J)), GRB.MAXIMIZE)
sub.optimize()

if sub.status == GRB.OPTIMAL:
    L_val = sub.ObjVal
    print(x.X)
else:
    print("No subproblem solution found.")

UB = min(UB, f @ y.X + a @ z.X + L_val)    
print(f"UB = {UB}")
print(f"LB = {LB}")
print(x.X)
d_star = []
x_star = []
while abs(UB - LB) >= tolerance:
    iteration = iteration + 1
    d_new = h + 40 * g.X 
    d_star.append(d_new)
    x_new = master.addMVar((I, J), vtype=GRB.CONTINUOUS)
    x_star.append(x_new)
    if L_val < GRB.INFINITY:
        master.addConstr(eta >=quicksum( x_new[i, j]*c[i,j]for i in range(I) for j in range(J)))
    for j in range(J):
        master.addConstr(quicksum(x_new[i, j] for i in range(I)) >= d_new[j])
    for i in range(I):
        master.addConstr(quicksum(x_new[i, j] for j in range(J)) <= z[i])
    master.optimize()
    if master.status == GRB.OPTIMAL:
        LB = max(LB, master.ObjVal)
    print(LB)
    z_star=z.X
    for c in sub.getConstrs():
        if c.ConstrName =="c1":
            sub.remove(c)
        if c.ConstrName =="c2":
            sub.remove(c)
    for i in range(I):
        sub.addConstr(gp.quicksum(x[i, j] for j in range(J)) <= z_star[i],name="c1")
        sub.addConstr(z_star[i]-quicksum(x[i, j] for j in range(J)) <=(1-gamma[i]*z_star[i]),name = "c2")
    sub.update()
    sub.optimize()

    if sub.status == GRB.OPTIMAL:
        L_val = sub.ObjVal
        UB = min(UB, f @ y.X + a @ z.X + L_val)   
    else:
        print("No subproblem solution found.")
        break
    print(f"UB = {UB}")
    print(f"LB = {LB}")

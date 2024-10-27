import gurobipy as gp
from gurobipy import GRB, quicksum
import numpy as np
import time

def solve_ccg_model(f, a, c, h, I, J,K,d_tilde,cap_gamma = 0.5, tolerance=1e-4, bigm=1e4):
    master = gp.Model("master_problem")
    master.Params.LogToConsole = 0
    y = master.addMVar((I,), vtype=GRB.BINARY)
    z = master.addMVar((I,), lb=0, vtype=GRB.INTEGER)

    eta = master.addVar(lb=0, vtype=GRB.CONTINUOUS)
    master.addConstr(z<=K*y)
    total_demand = np.sum(h+d_tilde)
    master.addConstr(quicksum(z[i] for i in range(I))>=total_demand)
    master.setObjective(f@y + a@z + eta, GRB.MINIMIZE)
    master.optimize()
    LB = -GRB.INFINITY
    UB = GRB.INFINITY
    tolerance = 1e-4
    iteration = 0
    if master.status == GRB.OPTIMAL:
        LB = max(LB, master.ObjVal)
    z_star = z.X
# Subproblem setup
    sub = gp.Model("sub_problem")
    sub.Params.LogToConsole = 0
    sub.Params.InfUnbdInfo = 1  # Enable infeasibility diagnostics

    # Subproblem variables
    alpha = sub.addMVar((I, J), vtype=GRB.BINARY, name="alpha")
    beta = sub.addMVar(J, vtype=GRB.BINARY, name="beta")
    gamma = sub.addMVar(I, vtype=GRB.BINARY, name="gamma")
    x = sub.addMVar((I, J), lb=0, vtype=GRB.CONTINUOUS, name="x")
    pi = sub.addMVar(I, lb=0, vtype=GRB.CONTINUOUS, name="pi")
    lambda_var = sub.addMVar(J, lb=0, vtype=GRB.CONTINUOUS, name="lambda")
    g = sub.addMVar(J, lb=0, ub=1, vtype=GRB.CONTINUOUS, name="g")
    d = h + g * d_tilde

    # Constraints with unique names
    for i in range(I):
        sub.addConstr(gp.quicksum(x[i, j] for j in range(J)) <= z_star[i], name=f"capacity_{i}")
        sub.addConstr(z_star[i] - gp.quicksum(x[i, j] for j in range(J)) <= (1 - gamma[i]) * z_star[i], name=f"gamma_{i}")
        sub.addConstr(pi[i] <= gamma[i] * bigm, name=f"pi_gamma_{i}")

    for j in range(J):
        sub.addConstr(gp.quicksum(x[i, j] for i in range(I)) >= d[j], name=f"demand_{j}")
        sub.addConstr(lambda_var[j] <= bigm * beta[j], name=f"lambda_beta_{j}")
        sub.addConstr(gp.quicksum(x[i, j] for i in range(I)) - d[j] <= d_tilde[j] * (1 - beta[j]), name=f"beta_constraint_{j}")

    for i in range(I):
        for j in range(J):
            sub.addConstr(lambda_var[j] - pi[i] <= c[i, j], name=f"dual_feasibility_{i}_{j}")
            sub.addConstr(x[i, j] <= bigm * alpha[i, j], name=f"x_alpha_{i}_{j}")
            sub.addConstr(c[i, j] - lambda_var[j] + pi[i] <= (c[i, j] + bigm) * (1 - alpha[i, j]), name=f"alpha_constraint_{i}_{j}")

    # Corrected uncertainty set constraint
    sub.addConstr(gp.quicksum(g[j] for j in range(J)) <= J * cap_gamma, name="uncertainty_set")

    # Objective function
    sub.setObjective(gp.quicksum(c[i, j] * x[i, j] for i in range(I) for j in range(J)), GRB.MAXIMIZE)

    # Optimize the subproblem
    sub.optimize()
    L_val = sub.ObjVal
    UB = min(UB, f @ y.X + a @ z.X + L_val)    
    d_star = []
    x_star = [] 

    while abs(UB - LB) >= tolerance:
        iteration = iteration + 1
        d_new = h + d_tilde * g.X 
        d_star.append(d_new)
        x_new = master.addMVar((I, J), lb=0, vtype=GRB.CONTINUOUS, name=f"x_{iteration}")
        x_star.append(x_new)
        if L_val < GRB.INFINITY:
            # Add cuts to the master problem
            master.addConstr(eta >= quicksum(c[i, j] * x_new[i, j] for i in range(I) for j in range(J)), name=f"eta_cut_{iteration}")

        # Add demand satisfaction constraints
        for j in range(J):
            master.addConstr(quicksum(x_new[i, j] for i in range(I)) >= d_new[j], name=f"demand_master_{iteration}_{j}")

        # Add capacity constraints
        for i in range(I):
            master.addConstr(quicksum(x_new[i, j] for j in range(J)) <= z[i], name=f"capacity_master_{iteration}_{i}")

        master.optimize()
        if master.status == GRB.OPTIMAL:
            LB = max(LB, master.ObjVal)
        z_star=z.X
        # Remove previous capacity constraints
        for i in range(I):
            sub.remove(sub.getConstrByName(f"capacity_{i}"))
            sub.remove(sub.getConstrByName(f"gamma_{i}"))
            sub.remove(sub.getConstrByName(f"pi_gamma_{i}"))
        sub.update()

        # Add updated capacity constraints with new z_star
        for i in range(I):
            sub.addConstr(gp.quicksum(x[i, j] for j in range(J)) <= z_star[i], name=f"capacity_{i}")
            sub.addConstr(z_star[i] - gp.quicksum(x[i, j] for j in range(J)) <= (1 - gamma[i]) * z_star[i], name=f"gamma_{i}")
            sub.addConstr(pi[i] <= gamma[i] * bigm, name=f"pi_gamma_{i}")

        sub.optimize()

        if sub.status == GRB.OPTIMAL:
            L_val = sub.ObjVal
            UB = min(UB, f @ y.X + a @ z.X + L_val)   
        else:
            print("No subproblem solution found.")
            break
        if iteration >= 10:
            break
        
    return {
        'LowerBound': LB,
        'UpperBound': UB,
        'y': y.X,
        'z': z.X,
        'eta': eta.X,
        'x': x.X,
        'Iterations': iteration
    }


def generate_instance(I, J):
    # Demand dj
    h = np.random.randint(10, 500, size=J)
    alpha = np.random.uniform(0.1, 0.5, size=J)
    d_tilde = (alpha * h).astype(int) 

    # Capacity Ki
    K = np.random.randint(200, 700, size=I)

    # Ensure feasibility: total capacity >= total demand
    total_demand = np.sum(h+d_tilde)
    for i in range(I):
        K[i] = max(K[i],total_demand/J)
    

    # Fixed cost fi
    f = np.random.uniform(100, 1000, size=I)

    # Unit capacity cost ai
    a = np.random.uniform(10, 100, size=I)

    # Transportation cost cij
    c = np.random.uniform(1, 1000, size=(I, J))

    # Return generated data
    return h, d_tilde, K, f, a, c
I = 10
J = 10
cap_gamma_values = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7,0.8,0.9,1]
results = []

for cap_gamma in cap_gamma_values:
    total_iterations = 0
    total_runtime = 0
    successful_runs = 0  # To account for any runs that might return None
    for instance_num in range(5):
        # Generate a new instance for each run
        h, d_tilde, K, f, a, c = generate_instance(I, J)

        start_time = time.time()
        result = solve_ccg_model(f, a, c, h, I, J, K, d_tilde, cap_gamma=cap_gamma)
        end_time = time.time()
        run_time = end_time - start_time

        if result:
            iteration_number = result['Iterations']
            total_iterations += iteration_number
            total_runtime += run_time
            successful_runs += 1
        else:
            print(f"Instance {instance_num+1} at cap_gamma {cap_gamma} failed to solve.")

    # Compute averages
    if successful_runs > 0:
        avg_iterations = total_iterations / successful_runs
        avg_runtime = total_runtime / successful_runs
    else:
        avg_iterations = None
        avg_runtime = None

    results.append({
        'Cap_Gamma': cap_gamma,
        'Avg_Iterations': avg_iterations,
        'Avg_RunTime': avg_runtime,
        'Successful_Runs': successful_runs
    })
    print(f"Cap_Gamma: {cap_gamma}, Avg Iterations: {avg_iterations}, Avg RunTime: {avg_runtime:.4f} seconds over {successful_runs} runs")

# Generate a table of results
print("\nSummary of Results:")
print("Cap_Gamma\tAvg_Iterations\tAvg_RunTime (seconds)\tSuccessful_Runs")
for res in results:
    print(f"{res['Cap_Gamma']}\t\t{res['Avg_Iterations']}\t\t{res['Avg_RunTime']:.4f}\t\t{res['Successful_Runs']}")
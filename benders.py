import gurobipy as gp
from gurobipy import GRB, quicksum
import numpy as np
import time

def solve_benders_decomposition(f, a, c, h, I, J, K, d_tilde, cap_gamma=0.5, tolerance=1e-2, bigm=1e4):
    # Initialize Master Problem
    master = gp.Model("master_problem")
    master.Params.LogToConsole = 0
    y = master.addMVar((I,), vtype=GRB.BINARY, name="y")
    z = master.addMVar((I,), lb=0, vtype=GRB.INTEGER, name="z")
    eta = master.addVar(lb=0, vtype=GRB.CONTINUOUS, name="eta")

    # Master Problem Constraints
    master.addConstr(z <= K * y, name="capacity_limit")
    total_demand = np.sum(h + d_tilde)
    master.addConstr(gp.quicksum(z[i] for i in range(I)) >= total_demand, name="total_demand")
    # Initialize bounds and iteration counter
    LB = -GRB.INFINITY
    UB = GRB.INFINITY
    iteration = 0
    # Master Problem Objective
    master.setObjective(f @ y + a @ z + eta, GRB.MINIMIZE)

    while abs(UB - LB) >= tolerance:
        if LB > UB:
            print("Warning: LB exceeds UB, adjusting LB to UB for consistency.")
            LB = UB
        iteration += 1
        master.optimize()
        if master.status != GRB.OPTIMAL:
            print("Master problem not optimal.")
            break

        z_val = z.X
        y_val = y.X

        LB = max(LB, master.ObjVal)

        # Subproblem: Fix y and z, solve the dual of the transportation problem
        sub = gp.Model("sub_problem")
        sub.Params.LogToConsole = 0

        # Subproblem Variables
        pi = sub.addMVar(I, lb=0, vtype=GRB.CONTINUOUS, name="pi")
        lambd = sub.addMVar(J, lb=0, vtype=GRB.CONTINUOUS, name="lambd")
        g = sub.addMVar(J, vtype=GRB.BINARY, name="g")
        w = sub.addMVar(J, lb=0, vtype=GRB.CONTINUOUS, name="w")

        # Subproblem Constraints
        for i in range(I):
            for j in range(J):
                sub.addConstr(lambd[j] - pi[i] <= c[i][j], name=f"dual_constraint_{i}_{j}")

        # Uncertainty budget constraint
        sub.addConstr(gp.quicksum(g[j] for j in range(J)) <= cap_gamma * J, name="uncertainty_budget")

        # Constraints involving w, lambd, and g
        for j in range(J):
            sub.addConstr(w[j] <= lambd[j], name=f"w_lambda_{j}")
            sub.addConstr(w[j] <= bigm * g[j], name=f"w_Mg_{j}")
            sub.addConstr(w[j] >= lambd[j] - bigm * (1 - g[j]), name=f"w_lambda_M1g_{j}")

        # Subproblem Objective
        sub.setObjective(
            gp.quicksum(h[j] * lambd[j] + d_tilde[j] * w[j] for j in range(J)) - gp.quicksum(z_val[i] * pi[i] for i in range(I)),
            GRB.MAXIMIZE
        )

        sub.optimize()

        if sub.status == GRB.OPTIMAL:
            # Subproblem is bounded, generate optimality cut
            sub_obj = sub.ObjVal
            UB = min(UB, f @ y_val+ a @ z_val + sub_obj) 
            # Get dual variables
            pi_val = pi.X
            lambd_val = lambd.X
            w_val = w.X
            # Compute constant term
            constant_term = sum(h[j] * lambd_val[j] + d_tilde[j] * w_val[j] for j in range(J))
            # Add optimality cut to master problem
            master.addConstr(
                eta >= constant_term - gp.quicksum(pi_val[i] * z[i] for i in range(I)),
                name=f"optimality_cut_{iteration}"
            )
        elif sub.status == GRB.UNBOUNDED:
            # Handle unbounded subproblem (feasibility cut)
            # Extract unbounded ray and formulate the feasibility cut appropriately
            unb_vars = sub.UnbdRay
            pi_unb = unb_vars[:I]
            # Build feasibility cut
            master.addConstr(
                gp.quicksum(pi_unb[i] * z[i] for i in range(I)) >= 0,
                name=f"feasibility_cut_{iteration}"
            )
        else:
            print("Subproblem status:", sub.status)
            break
        print("LB = ",LB)
        print("UB = ",UB)
        
    return {
        'LowerBound': LB,
        'UpperBound': UB,
        'y': y.X,
        'z': z.X,
        'eta': eta.X,
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
h, d_tilde, K, f, a, c = generate_instance(I, J)
solve_benders_decomposition(f, a, c, h, I, J, K, d_tilde)
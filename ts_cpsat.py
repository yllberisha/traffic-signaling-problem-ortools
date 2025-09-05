import sys
import os
from ortools.sat.python import cp_model

def main(input_file, output_dir):
    with open(input_file, 'r') as f:
        lines = f.readlines()

    line_idx = 0
    D, I, S_num, V, F = map(int, lines[line_idx].split())
    line_idx += 1

    streets = {}  # name to index
    B = [0] * S_num  # start intersection
    E = [0] * S_num  # end intersection
    L = [0] * S_num  # length
    in_streets = [[] for _ in range(I)]  # list of street indices incoming to i

    for s in range(S_num):
        parts = lines[line_idx].split()
        B[s] = int(parts[0])
        E[s] = int(parts[1])
        name = parts[2]
        L[s] = int(parts[3])
        streets[name] = s
        in_streets[E[s]].append(s)
        line_idx += 1

    car_paths = []  # list of list of street indices
    for v in range(V):
        parts = lines[line_idx].split()
        P = int(parts[0])
        path = [streets[name] for name in parts[1:]]
        car_paths.append(path)
        line_idx += 1

    # Events: crossing events
    events = []  # list of (car k, path pos l)
    street_events = [[] for _ in range(S_num)]  # per street, list of event indices e
    event_to_street = []
    for k in range(V):
        path = car_paths[k]
        for l in range(len(path) - 1):
            e = len(events)
            s = path[l]
            street_events[s].append(e)
            events.append((k, l))
            event_to_street.append(s)

    num_events = len(events)

    model = cp_model.CpModel()

    # Schedule variables
    max_cycle = D

    # Per intersection i
    y = [[model.NewBoolVar(f'y_{i}_{j}') for j in range(len(in_streets[i]))] for i in range(I)]
    T_var = [[model.NewIntVar(0, max_cycle, f'T_{i}_{j}') for j in range(len(in_streets[i]))] for i in range(I)]
    a_var = [[model.NewIntVar(0, max_cycle * len(in_streets[i]), f'a_{i}_{j}') for j in range(len(in_streets[i]))] for i in range(I)]
    x_var = [[[model.NewBoolVar(f'x_{i}_{j}_{p}') for p in range(len(in_streets[i]))] for j in range(len(in_streets[i]))] for i in range(I)]
    u_var = [[model.NewBoolVar(f'u_{i}_{p}') for p in range(len(in_streets[i]))] for i in range(I)]
    d_var = [[model.NewIntVar(0, max_cycle, f'd_{i}_{p}') for p in range(len(in_streets[i]))] for i in range(I)]
    b_var = [[model.NewIntVar(0, max_cycle * len(in_streets[i]) + 1, f'b_{i}_{p}') for p in range(len(in_streets[i]) + 1)] for i in range(I)]
    C_var = [model.NewIntVar(0, max_cycle * len(in_streets[i]), f'C_{i}') for i in range(I)]

    for i in range(I):
        in_count = len(in_streets[i])
        if in_count == 0:
            continue
        for j in range(in_count):
            model.Add(T_var[i][j] >= 1).OnlyEnforceIf(y[i][j])
            model.Add(T_var[i][j] == 0).OnlyEnforceIf(y[i][j].Not())
            model.Add(sum(x_var[i][j][p] for p in range(in_count)) == y[i][j])

        for p in range(in_count):
            model.Add(sum(x_var[i][j][p] for j in range(in_count)) == u_var[i][p])

            # Fix for d_var: sum_j x_{j,p} * T_j
            terms_d = []
            for j in range(in_count):
                term_d = model.NewIntVar(0, max_cycle, f'term_d_{i}_{p}_{j}')
                model.Add(term_d == 0).OnlyEnforceIf(x_var[i][j][p].Not())
                model.Add(term_d == T_var[i][j]).OnlyEnforceIf(x_var[i][j][p])
                terms_d.append(term_d)
            model.Add(d_var[i][p] == sum(terms_d))

        for p in range(in_count - 1):
            model.Add(u_var[i][p] >= u_var[i][p + 1])

        model.Add(b_var[i][0] == 0)
        for p in range(in_count):
            model.Add(b_var[i][p + 1] == b_var[i][p] + d_var[i][p])

        model.Add(C_var[i] == b_var[i][in_count])

        # Fix for a_var: sum_p x_{j,p} * b_p
        for j in range(in_count):
            terms_a = []
            for p in range(in_count):
                term_a = model.NewIntVar(0, max_cycle * in_count, f'term_a_{i}_{j}_{p}')
                model.Add(term_a == 0).OnlyEnforceIf(x_var[i][j][p].Not())
                model.Add(term_a == b_var[i][p]).OnlyEnforceIf(x_var[i][j][p])
                terms_a.append(term_a)
            model.Add(a_var[i][j] == sum(terms_a))

    # Car variables
    arr_var = [model.NewIntVar(0, D * V, f'arr_{e}') for e in range(num_events)]  # large upper
    tau_var = [model.NewIntVar(0, D * V, f'tau_{e}') for e in range(num_events)]
    z_var = [model.NewBoolVar(f'z_{e}') for e in range(num_events)]

    event_index = {}  # (k, l) -> e
    for e, (k, l) in enumerate(events):
        event_index[(k, l)] = e
        s = event_to_street[e]
        i = E[s]
        j = in_streets[i].index(s)
        model.Add(z_var[e] <= y[i][j])
        model.Add(tau_var[e] >= arr_var[e])
        if l == 0:
            model.Add(arr_var[e] == 0)
        else:
            prev_e = event_index[(k, l - 1)]
            s_prev_next = car_paths[k][l]
            model.Add(arr_var[e] == tau_var[prev_e] + L[s_prev_next])

    # Finishing and scoring
    fin_var = [model.NewIntVar(0, D * V, f'fin_{k}') for k in range(V)]
    f_var = [model.NewBoolVar(f'f_{k}') for k in range(V)]
    rscore_var = [model.NewIntVar(0, D, f'rscore_{k}') for k in range(V)]
    sk_var = [model.NewIntVar(0, F + D, f'sk_{k}') for k in range(V)]

    for k in range(V):
        P = len(car_paths[k])
        if P == 1:
            model.Add(fin_var[k] == 0)
            model.Add(f_var[k] == 1)
        else:
            last_l = P - 2
            last_e = event_index[(k, last_l)]
            model.Add(fin_var[k] == tau_var[last_e] + L[car_paths[k][P - 1]]).OnlyEnforceIf(f_var[k])
            for l in range(P - 1):
                model.Add(f_var[k] <= z_var[event_index[(k, l)]])

        model.Add(fin_var[k] <= D).OnlyEnforceIf(f_var[k])
        model.Add(fin_var[k] >= D + 1).OnlyEnforceIf(f_var[k].Not())
        model.Add(rscore_var[k] <= D * f_var[k])
        model.Add(rscore_var[k] <= D - fin_var[k]).OnlyEnforceIf(f_var[k])
        model.Add(rscore_var[k] >= D - fin_var[k] - D * (1 - f_var[k]))
        model.Add(sk_var[k] == F * f_var[k] + rscore_var[k])

    model.Maximize(sum(sk_var))

    # Service slots
    tau_s_var = [[model.NewIntVar(0, D * V, f'tau_s_{s}_{r}') for r in range(len(street_events[s]))] for s in range(S_num)]
    Q_s_var = [[model.NewIntVar(0, V, f'Q_s_{s}_{r}') for r in range(len(street_events[s]))] for s in range(S_num)]  # upper D//1
    R_s_var = [[model.NewIntVar(0, max_cycle, f'R_s_{s}_{r}') for r in range(len(street_events[s]))] for s in range(S_num)]
    A_var = [[[model.NewBoolVar(f'A_{s}_{local}_{r}') for r in range(len(street_events[s]))] for local in range(len(street_events[s]))] for s in range(S_num)]
    
    # Auxiliary variables for multiplication Q_s_var[s][r] * C_var[i]
    QC_product_var = [[model.NewIntVar(0, V * max_cycle * max(len(in_streets[i]) for i in range(I) if len(in_streets[i]) > 0), f'QC_product_{s}_{r}') for r in range(len(street_events[s]))] for s in range(S_num)]

    for s in range(S_num):
        N = len(street_events[s])
        if N == 0:
            continue
        i = E[s]
        j = in_streets[i].index(s)
        for r in range(N):
            # Create auxiliary variable for Q_s_var[s][r] * C_var[i]
            model.AddMultiplicationEquality(QC_product_var[s][r], Q_s_var[s][r], C_var[i])
            model.Add(tau_s_var[s][r] == a_var[i][j] + R_s_var[s][r] + QC_product_var[s][r]).OnlyEnforceIf(y[i][j])
            model.Add(R_s_var[s][r] <= T_var[i][j] - 1).OnlyEnforceIf(y[i][j])
            model.Add(R_s_var[s][r] >= 0)

        for r in range(1, N):
            model.Add(tau_s_var[s][r] >= tau_s_var[s][r - 1] + 1)

        for local, e in enumerate(street_events[s]):
            model.Add(sum(A_var[s][local][r] for r in range(N)) == z_var[e])

            # Fix for tau_var: sum_r A_r * tau_s_r
            terms_tau = []
            for r in range(N):
                term_tau = model.NewIntVar(0, D * V, f'term_tau_{s}_{local}_{r}')
                model.Add(term_tau == 0).OnlyEnforceIf(A_var[s][local][r].Not())
                model.Add(term_tau == tau_s_var[s][r]).OnlyEnforceIf(A_var[s][local][r])
                terms_tau.append(term_tau)
            model.Add(tau_var[e] == sum(terms_tau))

            for r in range(N):
                model.Add(arr_var[e] <= tau_s_var[s][r]).OnlyEnforceIf(A_var[s][local][r])

        for r in range(N):
            model.Add(sum(A_var[s][local][r] for local in range(N)) <= 1)

        # Optional strengthening
        B_var_s = [[model.NewBoolVar(f'B_{s}_{local}_{r}') for r in range(N)] for local in range(N)]

        for local in range(N):
            e = street_events[s][local]
            for r in range(N):
                B = B_var_s[local][r]
                model.Add(arr_var[e] <= tau_s_var[s][r]).OnlyEnforceIf(B)
                model.Add(arr_var[e] > tau_s_var[s][r]).OnlyEnforceIf(B.Not())

        for r in range(N):
            model.Add(sum(B_var_s[local][r] for local in range(N)) >= r + 1)

        # FIFO fix pairwise
        for idx1 in range(N):
            for idx2 in range(idx1 + 1, N):
                e1 = street_events[s][idx1]
                e2 = street_events[s][idx2]
                k1 = events[e1][0]
                k2 = events[e2][0]

                lt = model.NewBoolVar(f'lt_{e1}_{e2}')
                model.Add(arr_var[e1] < arr_var[e2]).OnlyEnforceIf(lt)
                model.Add(arr_var[e1] >= arr_var[e2]).OnlyEnforceIf(lt.Not())

                eq = model.NewBoolVar(f'eq_{e1}_{e2}')
                model.Add(arr_var[e1] == arr_var[e2]).OnlyEnforceIf(eq)
                model.Add(arr_var[e1] != arr_var[e2]).OnlyEnforceIf(eq.Not())

                # Tie break by car index (input order)
                tie_break = (k1 < k2)  # Assuming lower k is first in input
                bools = [lt]
                if tie_break:
                    bools.append(eq)

                is_before = model.NewBoolVar(f'is_before_{e1}_{e2}')
                model.AddBoolOr(bools).OnlyEnforceIf(is_before)
                model.AddBoolAnd([b.Not() for b in bools]).OnlyEnforceIf(is_before.Not())

                # If both cross, enforce tau order
                model.Add(tau_var[e1] < tau_var[e2]).OnlyEnforceIf([is_before, z_var[e1], z_var[e2]])
                model.Add(tau_var[e1] > tau_var[e2]).OnlyEnforceIf([is_before.Not(), z_var[e1], z_var[e2]])

    # Solve
    solver = cp_model.CpSolver()
    solver.parameters.max_time_in_seconds = 30000.0  # 5 min timeout or adjust
    status = solver.Solve(model)

    if status == cp_model.OPTIMAL or status == cp_model.FEASIBLE:
        # Extract schedule
        scheduled_intersections = []
        for i in range(I):
            in_count = len(in_streets[i])
            if in_count == 0 or solver.Value(C_var[i]) == 0:
                continue
            schedule = []
            for p in range(in_count):
                if solver.Value(u_var[i][p]) == 0:
                    continue
                for j in range(in_count):
                    if solver.Value(x_var[i][j][p]) == 1:
                        s = in_streets[i][j]
                        duration = solver.Value(T_var[i][j])
                        if duration > 0:
                            schedule.append((get_street_name(s, streets), duration))

            if schedule:
                scheduled_intersections.append((i, schedule))

        # Write output
        base_name = os.path.basename(input_file).split('.')[0]
        output_file = os.path.join(output_dir, f'{base_name}.out')
        with open(output_file, 'w') as f:
            f.write(f'{len(scheduled_intersections)}\n')
            for i, schedule in scheduled_intersections:
                f.write(f'{i}\n')
                f.write(f'{len(schedule)}\n')
                for name, T in schedule:
                    f.write(f'{name} {T}\n')

        print(f'Solution saved to {output_file}')
        print(f'Objective: {solver.ObjectiveValue()}')
    else:
        print('No solution found.')

def get_street_name(s, streets):
    for name, idx in streets.items():
        if idx == s:
            return name
    return None

if __name__ == '__main__':
    if len(sys.argv) != 3:
        print('Usage: python script.py <input_file> <output_dir>')
        sys.exit(1)
    main(sys.argv[1], sys.argv[2])
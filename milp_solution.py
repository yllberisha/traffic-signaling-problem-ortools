import sys
import os
from typing import List, Dict, Tuple
from ortools.linear_solver import pywraplp

# Data structures
class Street:
    __slots__ = ("start", "end", "name", "length", "id")
    def __init__(self, start: int, end: int, name: str, length: int, sid: int):
        self.start = start
        self.end = end
        self.name = name
        self.length = length
        self.id = sid

class Problem:
    def __init__(self):
        self.D = 0  # simulation duration (seconds)
        self.I = 0  # intersections
        self.S = 0  # streets
        self.V = 0  # cars
        self.F = 0  # bonus
        self.streets_by_name: Dict[str, Street] = {}
        self.streets: List[Street] = []
        self.incoming: Dict[int, List[int]] = {}
        self.outgoing: Dict[int, List[int]] = {}
        self.cars_routes: List[List[int]] = []  # list of street ids


def parse_input(path: str) -> Problem:
    with open(path, "r", encoding="utf-8") as f:
        it = iter(f.read().strip().split())
        pr = Problem()
        pr.D = int(next(it))
        pr.I = int(next(it))
        pr.S = int(next(it))
        pr.V = int(next(it))
        pr.F = int(next(it))

        pr.incoming = {i: [] for i in range(pr.I)}
        pr.outgoing = {i: [] for i in range(pr.I)}

        for s_id in range(pr.S):
            b = int(next(it))
            e = int(next(it))
            name = next(it)
            l = int(next(it))
            st = Street(b, e, name, l, s_id)
            pr.streets.append(st)
            pr.streets_by_name[name] = st
            pr.incoming[e].append(s_id)
            pr.outgoing[b].append(s_id)

        for _ in range(pr.V):
            p = int(next(it))
            route_names = [next(it) for __ in range(p)]
            route = [pr.streets_by_name[n].id for n in route_names]
            pr.cars_routes.append(route)

        return pr


def build_and_solve_milp(pr: Problem, time_limit_ms: int = 120000):
    D = pr.D
    solver = pywraplp.Solver.CreateSolver("CBC")
    if solver is None:
        raise RuntimeError("Failed to create CBC solver. Ensure ortools is installed.")
    if time_limit_ms is not None:
        solver.SetTimeLimit(time_limit_ms)

    # Variables
    # x[i][s][t] in {0,1}: 1 if at time t, intersection i gives green to incoming street s
    x = {}
    for i in range(pr.I):
        inc = pr.incoming[i]
        for s in inc:
            for t in range(D):
                x[(i, s, t)] = solver.BoolVar(f"x_{i}_{s}_{t}")

    # enter[(k,j,t)] in {0,1}: car k enters street route[j] at time t
    # c[(k,j,t)] in {0,1}: car k crosses intersection after completing street route[j] at time t (starts next at t+1)
    enter = {}
    c = {}

    # Precompute each car's route street lengths
    route_lengths: List[List[int]] = [[pr.streets[sid].length for sid in route] for route in pr.cars_routes]

    for k, route in enumerate(pr.cars_routes):
        m = len(route)
        for j in range(m):
            for t in range(D):
                enter[(k, j, t)] = solver.BoolVar(f"enter_{k}_{j}_{t}")
        for j in range(m - 1):
            for t in range(D):
                c[(k, j, t)] = solver.BoolVar(f"c_{k}_{j}_{t}")

    # Constraints
    # Exactly one incoming street green per intersection per second (if intersection has any incoming)
    for i in range(pr.I):
        inc = pr.incoming[i]
        for t in range(D):
            ct = solver.Sum(x[(i, s, t)] for s in inc) if inc else 0
            if inc:
                solver.Add(ct == 1, f"one_green_{i}_{t}")

    # Force starting condition: each car starts its first street at t=0
    for k, route in enumerate(pr.cars_routes):
        solver.Add(enter[(k, 0, 0)] == 1)
        for t in range(1, D):
            solver.Add(enter[(k, 0, t)] == 0)

    # Each stage j can be entered at most once (ensures single progression)
    for k, route in enumerate(pr.cars_routes):
        m = len(route)
        for j in range(m):
            solver.Add(solver.Sum(enter[(k, j, t)] for t in range(D)) <= 1)

    # Link crossing to next entry: immediate start on the same second
    for k, route in enumerate(pr.cars_routes):
        m = len(route)
        for j in range(m - 1):
            for t in range(D):
                solver.Add(enter[(k, j + 1, t)] == c[(k, j, t)])
        # Can't enter beyond horizon
        solver.Add(enter[(k, j + 1, 0)] == 0)

    # Crossing only after arrival and if green, one car served per street per second when green
    # For each (k,j,t): c[k,j,t] <= arrivals_by_t - served_before_t
    # arrivals_by_t = sum_{tau: tau+L_j <= t} enter[k,j,tau]
    # served_before_t = sum_{u < t} c[k,j,u]
    for k, route in enumerate(pr.cars_routes):
        m = len(route)
        for j in range(m - 1):
            L = route_lengths[k][j]
            street_id = route[j]
            inter = pr.streets[street_id].end
            for t in range(D):
                arrivals = []
                for tau in range(D):
                    if tau + L <= t:
                        arrivals.append(enter[(k, j, tau)])
                served_before = []
                for u in range(t):
                    served_before.append(c[(k, j, u)])
                if arrivals:
                    rhs = solver.Sum(arrivals) - (solver.Sum(served_before) if served_before else 0)
                    solver.Add(c[(k, j, t)] <= rhs)
                else:
                    solver.Add(c[(k, j, t)] == 0)
                # Must be green for this incoming street to cross
                solver.Add(c[(k, j, t)] <= x[(inter, street_id, t)])

    # Intersection service capacity: at most one car crosses from a street when green per second
    for i in range(pr.I):
        for s in pr.incoming[i]:
            relevant = [(k, j) for k, route in enumerate(pr.cars_routes) for j in range(len(route) - 1) if route[j] == s]
            for t in range(D):
                if relevant:
                    solver.Add(solver.Sum(c[(k, j, t)] for (k, j) in relevant) <= x[(i, s, t)])
                # If no relevant cars use this street, we leave x free so that equality above remains feasible.

    # Objective: maximize exact Hash Code score
    finished = []  # finished[k] == 1 if car k finishes within horizon
    last_enter_terms = []
    for k, route in enumerate(pr.cars_routes):
        m = len(route)
        L_last = route_lengths[k][m - 1]
        # Sum of entries to last street that finish within horizon (<= 1 due to single-entry constraint)
        valid_enters = []
        for t in range(D):
            finish_time = t + L_last
            if finish_time <= D:
                valid_enters.append(enter[(k, m - 1, t)])
                # secondary term: (D - finish_time)
                last_enter_terms.append(((D - finish_time), enter[(k, m - 1, t)]))
        fk = solver.BoolVar(f"finished_{k}")
        if valid_enters:
            solver.Add(fk == solver.Sum(valid_enters))
        else:
            # Cannot finish within horizon regardless, force 0
            solver.Add(fk == 0)
        finished.append(fk)

    # Maximize F * sum(finished) + solver.Sum((D - finish_time) for finishing cars)
    solver.Maximize(
        pr.F * solver.Sum(finished) + solver.Sum(coeff * var for coeff, var in last_enter_terms)
    )

    result_status = solver.Solve()

    if result_status not in (pywraplp.Solver.OPTIMAL, pywraplp.Solver.FEASIBLE):
        print("No feasible solution found.")
        return None

    # Build schedule from x variables, fill every second to create a D-length cycle, then compress
    schedule: Dict[int, List[Tuple[str, int]]] = {}
    for i in range(pr.I):
        inc = pr.incoming[i]
        if not inc:
            continue
        # Build per-second sequence of street ids of length D
        full_seq: List[int] = []
        for t in range(D):
            chosen = -1
            for s in inc:
                if x[(i, s, t)].solution_value() >= 0.5:
                    chosen = s
                    break
            if chosen == -1:
                # No green chosen at this second in the MILP; pick a fallback street
                chosen = inc[0]
            full_seq.append(chosen)
        # Compress consecutive equal streets to durations
        runs: List[Tuple[int, int]] = []  # (street_id, duration)
        prev = full_seq[0]
        dur = 1
        for sid in full_seq[1:]:
            if sid == prev:
                dur += 1
            else:
                runs.append((prev, dur))
                prev, dur = sid, 1
        runs.append((prev, dur))
        # Map to street names
        sched_lines: List[Tuple[str, int]] = []
        for sid, dur in runs:
            if dur > 0:
                sched_lines.append((pr.streets[sid].name, int(dur)))
        if sched_lines:
            schedule[i] = sched_lines

    return schedule


def write_schedule(path_out: str, schedule: Dict[int, List[Tuple[str, int]]]):
    intersections = [i for i, lines in schedule.items() if lines]
    with open(path_out, "w", encoding="utf-8") as f:
        f.write(str(len(intersections)) + "\n")
        for i in intersections:
            f.write(str(i) + "\n")
            f.write(str(len(schedule[i])) + "\n")
            for street_name, dur in schedule[i]:
                f.write(f"{street_name} {dur}\n")


def main():
    if len(sys.argv) < 2:
        print("Usage: pwsh -c 'python milp_solution.py <input_file> [time_limit_ms]'\nExample: python milp_solution.py inputs/a_example.in 30000")
        sys.exit(1)
    input_path = sys.argv[1]
    time_limit_ms = int(sys.argv[2]) if len(sys.argv) > 2 else 120000

    pr = parse_input(input_path)
    schedule = build_and_solve_milp(pr, time_limit_ms=time_limit_ms)
    if schedule is None:
        print("No solution.")
        sys.exit(2)

    out_dir = os.path.join(os.path.dirname(input_path), "..", "outputs")
    os.makedirs(out_dir, exist_ok=True)
    base = os.path.splitext(os.path.basename(input_path))[0]
    out_path = os.path.join(out_dir, base + ".out")
    write_schedule(out_path, schedule)
    print(f"Wrote schedule to {out_path}")


if __name__ == "__main__":
    main()

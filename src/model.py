"""
CP-SAT model for traffic signaling optimization.
"""
from ortools.sat.python import cp_model
from typing import List, Dict, Tuple, Any
from .utils import get_street_name


class TrafficSignalingModel:
    """CP-SAT model for traffic signaling optimization."""
    
    def __init__(self, problem_data: Dict, car_paths: List, street_data: Dict):
        self.problem_data = problem_data
        self.car_paths = car_paths
        self.street_data = street_data
        
        # Extract commonly used values
        self.D = problem_data['D']
        self.I = problem_data['I']
        self.S_num = problem_data['S_num']
        self.V = problem_data['V']
        self.F = problem_data['F']
        
        self.streets = street_data['streets']
        self.B = street_data['B']
        self.E = street_data['E']
        self.L = street_data['L']
        self.in_streets = street_data['in_streets']
        
        self.model = cp_model.CpModel()
        self.max_cycle = self.D
        
        # Will be initialized in create_model
        self.events = None
        self.street_events = None
        self.event_to_street = None
        self.event_index = None
        
    def create_events(self):
        """Create crossing events from car paths."""
        self.events = []
        self.street_events = [[] for _ in range(self.S_num)]
        self.event_to_street = []
        
        for k, path in enumerate(self.car_paths):
            for l in range(len(path) - 1):
                e = len(self.events)
                s = path[l]
                self.street_events[s].append(e)
                self.events.append((k, l))
                self.event_to_street.append(s)
        
        # Create event index mapping
        self.event_index = {}
        for e, (k, l) in enumerate(self.events):
            self.event_index[(k, l)] = e
    
    def create_intersection_variables(self):
        """Create variables and constraints for intersection scheduling."""
        # Per intersection i
        self.y = [[self.model.NewBoolVar(f'y_{i}_{j}') for j in range(len(self.in_streets[i]))] for i in range(self.I)]
        self.T_var = [[self.model.NewIntVar(0, self.max_cycle, f'T_{i}_{j}') for j in range(len(self.in_streets[i]))] for i in range(self.I)]
        self.a_var = [[self.model.NewIntVar(0, self.max_cycle * len(self.in_streets[i]), f'a_{i}_{j}') for j in range(len(self.in_streets[i]))] for i in range(self.I)]
        self.x_var = [[[self.model.NewBoolVar(f'x_{i}_{j}_{p}') for p in range(len(self.in_streets[i]))] for j in range(len(self.in_streets[i]))] for i in range(self.I)]
        self.u_var = [[self.model.NewBoolVar(f'u_{i}_{p}') for p in range(len(self.in_streets[i]))] for i in range(self.I)]
        self.d_var = [[self.model.NewIntVar(0, self.max_cycle, f'd_{i}_{p}') for p in range(len(self.in_streets[i]))] for i in range(self.I)]
        self.b_var = [[self.model.NewIntVar(0, self.max_cycle * len(self.in_streets[i]) + 1, f'b_{i}_{p}') for p in range(len(self.in_streets[i]) + 1)] for i in range(self.I)]
        self.C_var = [self.model.NewIntVar(0, self.max_cycle * len(self.in_streets[i]), f'C_{i}') for i in range(self.I)]

    def add_intersection_constraints(self):
        """Add constraints for intersection scheduling."""
        for i in range(self.I):
            in_count = len(self.in_streets[i])
            if in_count == 0:
                continue
                
            # Basic scheduling constraints
            for j in range(in_count):
                self.model.Add(self.T_var[i][j] >= 1).OnlyEnforceIf(self.y[i][j])
                self.model.Add(self.T_var[i][j] == 0).OnlyEnforceIf(self.y[i][j].Not())
                self.model.Add(sum(self.x_var[i][j][p] for p in range(in_count)) == self.y[i][j])

            for p in range(in_count):
                self.model.Add(sum(self.x_var[i][j][p] for j in range(in_count)) == self.u_var[i][p])

                # Duration calculation
                terms_d = []
                for j in range(in_count):
                    term_d = self.model.NewIntVar(0, self.max_cycle, f'term_d_{i}_{p}_{j}')
                    self.model.Add(term_d == 0).OnlyEnforceIf(self.x_var[i][j][p].Not())
                    self.model.Add(term_d == self.T_var[i][j]).OnlyEnforceIf(self.x_var[i][j][p])
                    terms_d.append(term_d)
                self.model.Add(self.d_var[i][p] == sum(terms_d))

            # Position ordering
            for p in range(in_count - 1):
                self.model.Add(self.u_var[i][p] >= self.u_var[i][p + 1])

            # Cumulative times
            self.model.Add(self.b_var[i][0] == 0)
            for p in range(in_count):
                self.model.Add(self.b_var[i][p + 1] == self.b_var[i][p] + self.d_var[i][p])

            self.model.Add(self.C_var[i] == self.b_var[i][in_count])

            # Activation times
            for j in range(in_count):
                terms_a = []
                for p in range(in_count):
                    term_a = self.model.NewIntVar(0, self.max_cycle * in_count, f'term_a_{i}_{j}_{p}')
                    self.model.Add(term_a == 0).OnlyEnforceIf(self.x_var[i][j][p].Not())
                    self.model.Add(term_a == self.b_var[i][p]).OnlyEnforceIf(self.x_var[i][j][p])
                    terms_a.append(term_a)
                self.model.Add(self.a_var[i][j] == sum(terms_a))

    def create_car_variables(self):
        """Create variables for car movement."""
        num_events = len(self.events)
        self.arr_var = [self.model.NewIntVar(0, self.D * self.V, f'arr_{e}') for e in range(num_events)]
        self.tau_var = [self.model.NewIntVar(0, self.D * self.V, f'tau_{e}') for e in range(num_events)]
        self.z_var = [self.model.NewBoolVar(f'z_{e}') for e in range(num_events)]

    def add_car_constraints(self):
        """Add constraints for car movement."""
        for e, (k, l) in enumerate(self.events):
            s = self.event_to_street[e]
            i = self.E[s]
            j = self.in_streets[i].index(s)
            
            self.model.Add(self.z_var[e] <= self.y[i][j])
            self.model.Add(self.tau_var[e] >= self.arr_var[e])
            
            if l == 0:
                self.model.Add(self.arr_var[e] == 0)
            else:
                prev_e = self.event_index[(k, l - 1)]
                s_prev_next = self.car_paths[k][l]
                self.model.Add(self.arr_var[e] == self.tau_var[prev_e] + self.L[s_prev_next])

    def create_scoring_variables(self):
        """Create variables for scoring."""
        self.fin_var = [self.model.NewIntVar(0, self.D * self.V, f'fin_{k}') for k in range(self.V)]
        self.f_var = [self.model.NewBoolVar(f'f_{k}') for k in range(self.V)]
        self.rscore_var = [self.model.NewIntVar(0, self.D, f'rscore_{k}') for k in range(self.V)]
        self.sk_var = [self.model.NewIntVar(0, self.F + self.D, f'sk_{k}') for k in range(self.V)]

    def add_scoring_constraints(self):
        """Add scoring constraints."""
        for k in range(self.V):
            P = len(self.car_paths[k])
            if P == 1:
                self.model.Add(self.fin_var[k] == 0)
                self.model.Add(self.f_var[k] == 1)
            else:
                last_l = P - 2
                last_e = self.event_index[(k, last_l)]
                self.model.Add(self.fin_var[k] == self.tau_var[last_e] + self.L[self.car_paths[k][P - 1]]).OnlyEnforceIf(self.f_var[k])
                for l in range(P - 1):
                    self.model.Add(self.f_var[k] <= self.z_var[self.event_index[(k, l)]])

            self.model.Add(self.fin_var[k] <= self.D).OnlyEnforceIf(self.f_var[k])
            self.model.Add(self.fin_var[k] >= self.D + 1).OnlyEnforceIf(self.f_var[k].Not())
            self.model.Add(self.rscore_var[k] <= self.D * self.f_var[k])
            self.model.Add(self.rscore_var[k] <= self.D - self.fin_var[k]).OnlyEnforceIf(self.f_var[k])
            self.model.Add(self.rscore_var[k] >= self.D - self.fin_var[k] - self.D * (1 - self.f_var[k]))
            self.model.Add(self.sk_var[k] == self.F * self.f_var[k] + self.rscore_var[k])

    def create_service_variables(self):
        """Create service slot variables."""
        self.tau_s_var = [[self.model.NewIntVar(0, self.D * self.V, f'tau_s_{s}_{r}') for r in range(len(self.street_events[s]))] for s in range(self.S_num)]
        self.Q_s_var = [[self.model.NewIntVar(0, self.V, f'Q_s_{s}_{r}') for r in range(len(self.street_events[s]))] for s in range(self.S_num)]
        self.R_s_var = [[self.model.NewIntVar(0, self.max_cycle, f'R_s_{s}_{r}') for r in range(len(self.street_events[s]))] for s in range(self.S_num)]
        self.A_var = [[[self.model.NewBoolVar(f'A_{s}_{local}_{r}') for r in range(len(self.street_events[s]))] for local in range(len(self.street_events[s]))] for s in range(self.S_num)]
        
        max_in_streets = max(len(self.in_streets[i]) for i in range(self.I) if len(self.in_streets[i]) > 0) if any(len(self.in_streets[i]) > 0 for i in range(self.I)) else 1
        self.QC_product_var = [[self.model.NewIntVar(0, self.V * self.max_cycle * max_in_streets, f'QC_product_{s}_{r}') for r in range(len(self.street_events[s]))] for s in range(self.S_num)]

    def add_service_constraints(self):
        """Add service slot constraints."""
        for s in range(self.S_num):
            N = len(self.street_events[s])
            if N == 0:
                continue
                
            i = self.E[s]
            j = self.in_streets[i].index(s)
            
            for r in range(N):
                self.model.AddMultiplicationEquality(self.QC_product_var[s][r], self.Q_s_var[s][r], self.C_var[i])
                self.model.Add(self.tau_s_var[s][r] == self.a_var[i][j] + self.R_s_var[s][r] + self.QC_product_var[s][r]).OnlyEnforceIf(self.y[i][j])
                self.model.Add(self.R_s_var[s][r] <= self.T_var[i][j] - 1).OnlyEnforceIf(self.y[i][j])
                self.model.Add(self.R_s_var[s][r] >= 0)

            for r in range(1, N):
                self.model.Add(self.tau_s_var[s][r] >= self.tau_s_var[s][r - 1] + 1)

            self.add_assignment_constraints(s, N)
            self.add_strengthening_constraints(s, N)
            self.add_fifo_constraints(s, N)

    def add_assignment_constraints(self, s: int, N: int):
        """Add assignment constraints for service slots."""
        for local, e in enumerate(self.street_events[s]):
            self.model.Add(sum(self.A_var[s][local][r] for r in range(N)) == self.z_var[e])

            # Service time calculation
            terms_tau = []
            for r in range(N):
                term_tau = self.model.NewIntVar(0, self.D * self.V, f'term_tau_{s}_{local}_{r}')
                self.model.Add(term_tau == 0).OnlyEnforceIf(self.A_var[s][local][r].Not())
                self.model.Add(term_tau == self.tau_s_var[s][r]).OnlyEnforceIf(self.A_var[s][local][r])
                terms_tau.append(term_tau)
            self.model.Add(self.tau_var[e] == sum(terms_tau))

            for r in range(N):
                self.model.Add(self.arr_var[e] <= self.tau_s_var[s][r]).OnlyEnforceIf(self.A_var[s][local][r])

        for r in range(N):
            self.model.Add(sum(self.A_var[s][local][r] for local in range(N)) <= 1)

    def add_strengthening_constraints(self, s: int, N: int):
        """Add strengthening constraints."""
        B_var_s = [[self.model.NewBoolVar(f'B_{s}_{local}_{r}') for r in range(N)] for local in range(N)]

        for local in range(N):
            e = self.street_events[s][local]
            for r in range(N):
                B = B_var_s[local][r]
                self.model.Add(self.arr_var[e] <= self.tau_s_var[s][r]).OnlyEnforceIf(B)
                self.model.Add(self.arr_var[e] > self.tau_s_var[s][r]).OnlyEnforceIf(B.Not())

        for r in range(N):
            self.model.Add(sum(B_var_s[local][r] for local in range(N)) >= r + 1)

    def add_fifo_constraints(self, s: int, N: int):
        """Add FIFO constraints."""
        for idx1 in range(N):
            for idx2 in range(idx1 + 1, N):
                e1 = self.street_events[s][idx1]
                e2 = self.street_events[s][idx2]
                k1 = self.events[e1][0]
                k2 = self.events[e2][0]

                lt = self.model.NewBoolVar(f'lt_{e1}_{e2}')
                self.model.Add(self.arr_var[e1] < self.arr_var[e2]).OnlyEnforceIf(lt)
                self.model.Add(self.arr_var[e1] >= self.arr_var[e2]).OnlyEnforceIf(lt.Not())

                eq = self.model.NewBoolVar(f'eq_{e1}_{e2}')
                self.model.Add(self.arr_var[e1] == self.arr_var[e2]).OnlyEnforceIf(eq)
                self.model.Add(self.arr_var[e1] != self.arr_var[e2]).OnlyEnforceIf(eq.Not())

                tie_break = (k1 < k2)
                bools = [lt]
                if tie_break:
                    bools.append(eq)

                is_before = self.model.NewBoolVar(f'is_before_{e1}_{e2}')
                self.model.AddBoolOr(bools).OnlyEnforceIf(is_before)
                self.model.AddBoolAnd([b.Not() for b in bools]).OnlyEnforceIf(is_before.Not())

                self.model.Add(self.tau_var[e1] < self.tau_var[e2]).OnlyEnforceIf([is_before, self.z_var[e1], self.z_var[e2]])
                self.model.Add(self.tau_var[e1] > self.tau_var[e2]).OnlyEnforceIf([is_before.Not(), self.z_var[e1], self.z_var[e2]])

    def build_model(self):
        """Build the complete CP-SAT model."""
        self.create_events()
        
        self.create_intersection_variables()
        self.add_intersection_constraints()
        
        self.create_car_variables()
        self.add_car_constraints()
        
        self.create_scoring_variables()
        self.add_scoring_constraints()
        
        self.create_service_variables()
        self.add_service_constraints()
        
        # Set objective
        self.model.Maximize(sum(self.sk_var))

    def solve(self, time_limit: float = 300.0, num_threads: int = 8) -> Tuple[int, Any]:
        """
        Solve the model and return status and solver.
        
        Args:
            time_limit: Solver time limit in seconds
            num_threads: Number of threads for parallel solving
        """
        solver = cp_model.CpSolver()
        
        # Set solver parameters
        solver.parameters.max_time_in_seconds = time_limit
        solver.parameters.num_search_workers = num_threads
        
        # Enable parallel search strategies
        solver.parameters.search_branching = cp_model.PORTFOLIO_SEARCH
        solver.parameters.cp_model_presolve = True
        solver.parameters.cp_model_probing_level = 2
        
        # Enable logging for better visibility
        solver.parameters.log_search_progress = True
        solver.parameters.enumerate_all_solutions = False
        
        print(f"Solver configured with {num_threads} threads")
        print(f"Starting solve with {time_limit}s time limit...")
        
        status = solver.Solve(self.model)
        
        # Print solver statistics
        print(f"Solver status: {solver.StatusName(status)}")
        print(f"Solve time: {solver.WallTime():.2f} seconds")
        print(f"Branches explored: {solver.NumBranches()}")
        print(f"Conflicts found: {solver.NumConflicts()}")
        
        return status, solver

    def extract_solution(self, solver: Any) -> List[Tuple]:
        """Extract solution from solved model."""
        scheduled_intersections = []
        
        for i in range(self.I):
            in_count = len(self.in_streets[i])
            if in_count == 0 or solver.Value(self.C_var[i]) == 0:
                continue
                
            schedule = []
            for p in range(in_count):
                if solver.Value(self.u_var[i][p]) == 0:
                    continue
                for j in range(in_count):
                    if solver.Value(self.x_var[i][j][p]) == 1:
                        s = self.in_streets[i][j]
                        duration = solver.Value(self.T_var[i][j])
                        if duration > 0:
                            street_name = get_street_name(s, self.streets)
                            schedule.append((street_name, duration))

            if schedule:
                scheduled_intersections.append((i, schedule))
        
        return scheduled_intersections

import sys
import os
from collections import deque, defaultdict
from typing import Dict, List, Tuple

# Hash Code 2021 - Traffic Signaling
# Validator and scorer for a given input and schedule output

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
        self.D = 0
        self.I = 0
        self.S = 0
        self.V = 0
        self.F = 0
        self.streets_by_name: Dict[str, Street] = {}
        self.streets: List[Street] = []
        self.incoming: Dict[int, List[int]] = {}
        self.outgoing: Dict[int, List[int]] = {}
        self.cars_routes: List[List[int]] = []


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


def parse_schedule(path: str, pr: Problem) -> Dict[int, List[int]]:
    if not os.path.exists(path):
        raise FileNotFoundError(path)
    with open(path, "r", encoding="utf-8") as f:
        data = [line.strip() for line in f if line.strip()]
    if not data:
        return {}
    idx = 0
    try:
        A = int(data[idx]); idx += 1
    except Exception:
        raise ValueError("Invalid schedule header")

    schedule: Dict[int, List[int]] = defaultdict(list)
    for _ in range(A):
        if idx >= len(data):
            break
        i = int(data[idx]); idx += 1
        if idx >= len(data):
            break
        E = int(data[idx]); idx += 1
        for __ in range(E):
            if idx >= len(data):
                break
            parts = data[idx].split()
            idx += 1
            if len(parts) != 2:
                continue
            name, dur_s = parts
            try:
                dur = int(dur_s)
            except Exception:
                continue
            st = pr.streets_by_name.get(name)
            if st is None:
                continue
            if st.end != i:
                # Street does not end at this intersection; ignore
                continue
            if dur <= 0:
                continue
            schedule[i].extend([st.id] * dur)
    return schedule


def simulate_and_score(pr: Problem, schedule: Dict[int, List[int]]) -> Tuple[int, int, int]:
    D, F = pr.D, pr.F
    V = pr.V
    # Queues indexed by incoming street id
    queues: Dict[int, deque] = {sid: deque() for sid in range(pr.S)}
    # Arrival buckets: at time t, list of (car k, stage j) that arrive at end of street j
    # Only store arrivals for non-final stages (j < m-1). Final arrivals are counted as finished immediately.
    arrivals: List[List[Tuple[int, int]]] = [[] for _ in range(D + 1)]

    finish_time = [-1] * V

    # Initialize: each car starts on first street at t=0
    for k, route in enumerate(pr.cars_routes):
        m = len(route)
        first_len = pr.streets[route[0]].length
        t_arr = first_len
        if m == 1:
            # Finishes at end of first street
            if t_arr <= D:
                finish_time[k] = t_arr
            continue
        if t_arr <= D:
            arrivals[t_arr].append((k, 0))  # arrived at end of stage j=0

    # For fast intersection lookup: map street id -> intersection id where it ends
    street_end = [st.end for st in pr.streets]

    # Simulate each second
    for t in range(D):
        # 1) Add arrivals to queues
        for k, j in arrivals[t]:
            s_id = pr.cars_routes[k][j]
            queues[s_id].append((k, j))

        # 2) Serve one car per intersection according to schedule
        for i in range(pr.I):
            cyc = schedule.get(i)
            if not cyc:
                continue
            s_green = cyc[t % len(cyc)]
            if street_end[s_green] != i:
                # Defensive: skip if inconsistent
                continue
            q = queues[s_green]
            if not q:
                continue
            k, j = q.popleft()
            route = pr.cars_routes[k]
            m = len(route)
            # Move to next stage
            next_j = j + 1
            # If next_j == m: should not happen (we never queue after last street)
            if next_j >= m:
                continue
            next_street = route[next_j]
            t_finish_next = t + pr.streets[next_street].length
            if next_j == m - 1:
                # This arrival completes the route
                if t_finish_next <= D and finish_time[k] == -1:
                    finish_time[k] = t_finish_next
            else:
                if t_finish_next <= D:
                    arrivals[t_finish_next].append((k, next_j))
                # else: arrival beyond horizon doesn't affect score

    # Compute score
    finished = sum(1 for tfin in finish_time if tfin != -1)
    score = sum(F + (D - tfin) for tfin in finish_time if tfin != -1)
    return score, finished, V


def main():
    if len(sys.argv) < 3:
        print("Usage: python validate_and_score.py <input_file> <output_schedule_file>")
        sys.exit(1)
    input_path = sys.argv[1]
    output_path = sys.argv[2]

    pr = parse_input(input_path)
    schedule = parse_schedule(output_path, pr)
    score, finished, total = simulate_and_score(pr, schedule)

    print(f"Finished cars: {finished}/{total}")
    print(f"Score: {score}")


if __name__ == "__main__":
    main()

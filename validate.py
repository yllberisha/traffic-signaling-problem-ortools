import sys
from collections import defaultdict
import heapq

class Car:
    def __init__(self, id, path):
        self.id = id
        self.path = path
        self.current_idx = 0

def read_input(input_file):
    with open(input_file) as f:
        lines = f.read().splitlines()
    idx = 0
    D, I, S, V, F = map(int, lines[idx].split())
    idx += 1
    street_name_to_id = {}
    B_list = []
    E_list = []
    L_list = []
    for street_id in range(S):
        b, e, name, l = lines[idx].split()
        b, e, l = int(b), int(e), int(l)
        street_name_to_id[name] = street_id
        B_list.append(b)
        E_list.append(e)
        L_list.append(l)
        idx += 1
    cars = []
    for vid in range(V):
        parts = lines[idx].split()
        p = int(parts[0])
        path = [street_name_to_id[name] for name in parts[1:]]
        cars.append(Car(vid, path))
        idx += 1
    return D, I, S, V, F, B_list, E_list, L_list, cars, street_name_to_id

def read_submission(sub_file, street_name_to_id, I):
    schedules = [None] * I
    with open(sub_file) as f:
        lines = f.read().splitlines()
    idx = 0
    A = int(lines[idx])
    idx += 1
    for _ in range(A):
        i = int(lines[idx])
        idx += 1
        Ei = int(lines[idx])
        idx += 1
        sch = []
        cycle = 0
        offset = 0
        prefixes = []
        for __ in range(Ei):
            name, t = lines[idx].split()
            t = int(t)
            sid = street_name_to_id[name]
            sch.append((sid, t))
            prefixes.append((sid, t, offset))
            offset += t
            cycle += t
            idx += 1
        if cycle > 0:
            schedules[i] = (sch, cycle, prefixes)
    return schedules

def get_green_at_t(i, t, schedules):
    sch = schedules[i]
    if not sch:
        return None
    _, cycle, prefixes = sch
    if cycle == 0:
        return None
    pos = t % cycle
    for sid, dur, off in prefixes:
        if off <= pos < off + dur:
            return sid
    return None  # Should not reach here

def simulate(D, I, S, V, F, B, E, L, cars, schedules):
    queues = [[] for _ in range(S)]  # heaps per street
    active = set()
    score = 0
    # Initialize queues
    for car in cars:
        s = car.path[0]
        heapq.heappush(queues[s], (0, car.id, car))
        active.add(s)
    for t in range(D):
        curr_active = list(active)
        for s in curr_active:
            green = get_green_at_t(E[s], t, schedules)
            if green == s:
                queue = queues[s]
                if queue:
                    arrival, cid, car = queue[0]
                    if arrival <= t:
                        heapq.heappop(queue)
                        # Car crosses at t
                        car.current_idx += 1
                        if car.current_idx == len(car.path) - 1:
                            # Finish after last street
                            last_l = L[car.path[-1]]
                            finish = t + last_l
                            if finish <= D:
                                score += F + (D - finish)
                        else:
                            # Enter next street, arrive at its end
                            next_s = car.path[car.current_idx]
                            next_l = L[next_s]
                            next_arr = t + next_l
                            heapq.heappush(queues[next_s], (next_arr, car.id, car))
                            active.add(next_s)
                        if not queue:
                            active.discard(s)
    return score

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python score.py <input_file> <submission_file>")
        sys.exit(1)
    input_file = sys.argv[1]
    sub_file = sys.argv[2]
    D, I, S, V, F, B, E, L, cars, name_to_id = read_input(input_file)
    schedules = read_submission(sub_file, name_to_id, I)
    score = simulate(D, I, S, V, F, B, E, L, cars, schedules)
    print(score)
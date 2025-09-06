import sys
from collections import defaultdict
import heapq
import time

class Car:
    def __init__(self, id, path):
        self.id = id
        self.path = path
        self.current_idx = 0
        self.arrived = False
        self.score = 0
        self.commute_time = 0

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
    total_cycles = 0
    total_green_time = 0
    num_schedules = 0
    
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
            if name in street_name_to_id:
                sid = street_name_to_id[name]
                sch.append((sid, t))
                prefixes.append((sid, t, offset))
                offset += t
                cycle += t
                total_green_time += t
                num_schedules += 1
            idx += 1
        if cycle > 0:
            schedules[i] = (sch, cycle, prefixes)
            total_cycles += cycle
    
    avg_green_cycles = total_green_time / num_schedules if num_schedules > 0 else 0
    avg_total_cycles = total_cycles / A if A > 0 else 0
    
    return schedules, avg_green_cycles, avg_total_cycles

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
    start_time = time.time()
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
                                car.arrived = True
                                car.score = F + (D - finish)
                                car.commute_time = finish
                                score += car.score
                        else:
                            # Enter next street, arrive at its end
                            next_s = car.path[car.current_idx]
                            next_l = L[next_s]
                            next_arr = t + next_l
                            heapq.heappush(queues[next_s], (next_arr, car.id, car))
                            active.add(next_s)
                        if not queue:
                            active.discard(s)
    
    simulation_time = time.time() - start_time
    return score, cars, simulation_time

def create_insights(D, I, S, V, F, cars, avg_green_cycles, avg_total_cycles, score):
    arrived_cars = [car for car in cars if car.arrived]
    arrived_cars.sort(key=lambda x: x.commute_time)
    
    num_arrived_cars = len(arrived_cars)
    early_arrival_bonus = sum(car.score - F for car in arrived_cars)
    
    # Calculate statistics
    arrival_percentage = (num_arrived_cars / V) * 100 if V > 0 else 0
    
    insights = []
    
    # Score summary
    insights.append(f"The submission scored {score:,} points. This is the sum of")
    insights.append(f"{F * num_arrived_cars:,} bonus points for cars arriving before the deadline")
    insights.append(f"({F:,} points each) and {early_arrival_bonus:,} points for early arrival times.")
    insights.append("")
    
    # Arrival statistics
    insights.append(f"{num_arrived_cars:,} of {V:,} cars arrived before the deadline ({arrival_percentage:.0f}%).")
    
    if arrived_cars:
        avg_commute_time = sum(car.commute_time for car in arrived_cars) / num_arrived_cars
        insights.append(f"The earliest car arrived at its destination after {arrived_cars[0].commute_time:,}")
        insights.append(f"seconds scoring {arrived_cars[0].score:,} points, whereas the last")
        insights.append(f"car arrived at its destination after {arrived_cars[-1].commute_time:,}")
        insights.append(f"seconds scoring {arrived_cars[-1].score:,} points.")
        insights.append(f"Cars that arrived within the deadline drove for an average of {avg_commute_time:.2f}")
        insights.append("seconds to arrive at their destination.")
    insights.append("")
    
    # Traffic light statistics
    insights.append(f"The schedules for the {I:,} traffic lights had an average total cycle length of")
    insights.append(f"{avg_total_cycles:.2f} seconds. A traffic light that turned green was scheduled")
    insights.append(f"to stay green for {avg_green_cycles:.2f} seconds on average.")
    
    return "\n".join(insights)

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python validate.py <input_file> <submission_file>")
        sys.exit(1)
    
    input_file = sys.argv[1]
    sub_file = sys.argv[2]
    
    try:
        # Parse input data
        D, I, S, V, F, B, E, L, cars, name_to_id = read_input(input_file)
        
        # Parse submission data
        schedules, avg_green_cycles, avg_total_cycles = read_submission(sub_file, name_to_id, I)
        
        # Run simulation
        score, cars, simulation_time = simulate(D, I, S, V, F, B, E, L, cars, schedules)
        
        # Generate insights
        insights = create_insights(D, I, S, V, F, cars, avg_green_cycles, avg_total_cycles, score)
        
        # Print results
        print(insights)
        print(f"\nFinal Score: {score:,}")
        print(f"Simulation completed in {simulation_time:.2f} seconds")
        
    except Exception as e:
        print(f"Error: {str(e)}")
        sys.exit(1)
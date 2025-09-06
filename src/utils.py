"""
Utility functions for traffic signaling problem.
"""
import os
from typing import List, Dict, Tuple


def parse_input(input_file: str) -> Tuple[Dict, List, Dict]:
    """
    Parse the input file and return problem data.
    
    Returns:
        problem_data: Dictionary containing D, I, S_num, V, F
        car_paths: List of car paths (list of street indices)
        street_data: Dictionary containing streets info and graph structure
    """
    with open(input_file, 'r') as f:
        lines = f.readlines()

    line_idx = 0
    D, I, S_num, V, F = map(int, lines[line_idx].split())
    line_idx += 1

    # Street data
    streets = {}  # name to index
    B = [0] * S_num  # start intersection
    E = [0] * S_num  # end intersection
    L = [0] * S_num  # length
    in_streets = [[] for _ in range(I)]  # list of street indices incoming to intersection i

    for s in range(S_num):
        parts = lines[line_idx].split()
        B[s] = int(parts[0])
        E[s] = int(parts[1])
        name = parts[2]
        L[s] = int(parts[3])
        streets[name] = s
        in_streets[E[s]].append(s)
        line_idx += 1

    # Car paths
    car_paths = []
    for v in range(V):
        parts = lines[line_idx].split()
        P = int(parts[0])
        path = [streets[name] for name in parts[1:]]
        car_paths.append(path)
        line_idx += 1

    problem_data = {
        'D': D, 'I': I, 'S_num': S_num, 'V': V, 'F': F
    }
    
    street_data = {
        'streets': streets,
        'B': B, 'E': E, 'L': L,
        'in_streets': in_streets
    }
    
    return problem_data, car_paths, street_data


def create_events(car_paths: List[List[int]], S_num: int) -> Tuple[List, List, List]:
    """
    Create crossing events from car paths.
    
    Returns:
        events: List of (car k, path position l) tuples
        street_events: Per street, list of event indices
        event_to_street: Mapping from event to street
    """
    events = []
    street_events = [[] for _ in range(S_num)]
    event_to_street = []
    
    for k, path in enumerate(car_paths):
        for l in range(len(path) - 1):
            e = len(events)
            s = path[l]
            street_events[s].append(e)
            events.append((k, l))
            event_to_street.append(s)
    
    return events, street_events, event_to_street


def get_street_name(street_idx: int, streets: Dict[str, int]) -> str:
    """Get street name from street index."""
    for name, idx in streets.items():
        if idx == street_idx:
            return name
    return None


def write_solution(scheduled_intersections: List[Tuple], input_file: str, 
                  output_dir: str, streets: Dict[str, int], objective_value: float):
    """Write the solution to output file."""
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
    print(f'Objective: {objective_value}')

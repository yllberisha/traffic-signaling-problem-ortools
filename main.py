"""Traffic signaling optimization solver."""
import argparse
import os
import sys
from ortools.sat.python import cp_model
from src.utils import parse_input, write_solution
from src.model import TrafficSignalingModel


def solve(input_file: str, output_dir: str, time_limit: float = 300.0, threads: int = 8):
    """Solve traffic signaling optimization problem."""
    problem_data, car_paths, street_data = parse_input(input_file)
    model = TrafficSignalingModel(problem_data, car_paths, street_data)
    model.build_model()
    
    status, solver = model.solve(time_limit, threads)
    
    # Output results
    if status in [cp_model.OPTIMAL, cp_model.FEASIBLE]:
        solution = model.extract_solution(solver)
        write_solution(solution, input_file, output_dir, street_data['streets'], solver.ObjectiveValue())
    else:
        print('No solution found.')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Traffic signaling optimization')
    parser.add_argument('input_file', help='Input file path')
    parser.add_argument('output_dir', nargs='?', default='outputs', help='Output directory (default: outputs)')
    parser.add_argument('-t', '--time', type=float, default=300.0, help='Time limit (seconds)')
    parser.add_argument('-j', '--threads', type=int, default=8, help='Number of threads')
    args = parser.parse_args()
    
    # Quick validation and solve
    if not os.path.exists(args.input_file):
        sys.exit(f"Error: Input file '{args.input_file}' not found")
    if args.threads < 1 or args.time <= 0:
        sys.exit("Error: Invalid parameters")
    
    os.makedirs(args.output_dir, exist_ok=True)
    solve(args.input_file, args.output_dir, args.time, args.threads)

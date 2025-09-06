# Traffic Signaling Problem Solver using OR-Tools

This repository contains a solution to the Traffic Signaling optimization problem using Google OR-Tools CP-SAT solver. The problem was originally from Google Hash Code 2021 Qualification Round.

## Problem Description

The Traffic Signaling problem involves optimizing traffic light schedules at intersections to minimize the total travel time of cars through a city. Each car follows a predetermined path through the city streets, and the goal is to coordinate traffic lights to allow cars to complete their journeys as efficiently as possible.

### Key Components:
- **Intersections**: Points where multiple streets meet, controlled by traffic lights
- **Streets**: One-way roads connecting intersections, each with a travel time
- **Cars**: Vehicles following specific paths through the city
- **Traffic Lights**: Control which street has a green light at each intersection

## Repository Structure

```
├── main.py                   # Main entry point for solving problems
├── validate.py               # Solution validation script
├── src/                      # Core implementation
│   ├── __init__.py
│   ├── model.py              # CP-SAT model implementation
│   └── utils.py              # Input parsing and output utilities
├── inputs/                   # Problem instances
├── outputs/                  # Generated solutions
└── docs/                     # Documentation and problem statements
    ├── hashcode_2021_qualification_round.pdf
    ├── hashcode_2021_qualification_round_scoreboard.json
    └── traffic_signaling_CPSAT.pdf
```

## Requirements

- Python 3.10+
- Google OR-Tools 9.14.6206

## Installation

1. **Clone the repository:**
   ```bash
   git clone https://github.com/yllberisha/traffic-signaling-problem-ortools.git
   cd traffic-signaling-problem-ortools
   ```

2. **Create and activate a conda environment:**
   ```bash
   conda create -n pyortools_env python=3.10
   conda activate pyortools_env
   ```

3. **Install required packages:**
   ```bash
   pip install ortools==9.14.6206
   ```

## Usage

### Basic Usage

Run the solver on a problem instance:

```bash
python main.py inputs/a_example.in
```

This will generate a solution file in the `outputs/` directory.

**Parameters:**
- `input_file`: Path to the input problem file (required)
- `output_dir`: Output directory for solutions (default: `outputs`)
- `-t, --time`: Time limit in seconds (default: 300)
- `-j, --threads`: Number of threads to use (default: 8)

### Solution Validation

Validate a generated solution:

```bash
python validate.py inputs/a_example.in outputs/a_example.out
```

## Input Format

Input files contain:
1. **Header**: `D I S V F` where:
   - `D`: Duration of the simulation
   - `I`: Number of intersections
   - `S`: Number of streets
   - `V`: Number of cars
   - `F`: Bonus points for cars finishing before time D

2. **Streets**: `S` lines with `B E name L` where:
   - `B`: Start intersection
   - `E`: End intersection
   - `name`: Street name
   - `L`: Time to travel the street

3. **Cars**: `V` lines with `P street1 street2 ...` where:
   - `P`: Number of streets in the car's path
   - `street1, street2, ...`: Street names in the car's path

## Output Format

Output files contain:
1. **Number of intersections** with scheduled traffic lights
2. **For each intersection**:
   - Intersection ID
   - Number of streets with green light schedules
   - For each street: street name and green light duration

## Algorithm

The solver uses Google OR-Tools CP-SAT (Constraint Programming with SAT) to model and solve the traffic optimization problem. The key components include:

1. **Variables**: 
   - Start times for crossing events
   - Green light schedules for intersections
   - Car completion times

2. **Constraints**:
   - Traffic light scheduling constraints
   - Car movement constraints
   - Timing and sequencing constraints

3. **Objective**: Maximize the total score (cars completing their journeys + bonus points)


This project is open source and available under the [MIT License](LICENSE).

## References

- [Google Hash Code 2021 Problem Statement](docs/hashcode_2021_qualification_round.pdf)
- [Google OR-Tools Documentation](https://developers.google.com/optimization)
- [CP-SAT Solver Guide](https://developers.google.com/optimization/cp/cp_solver)

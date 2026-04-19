# Lane-Aware Multi-Robot Traffic Control System

A simulation framework for coordinating multiple autonomous robots navigating a shared lane network. The system handles path planning, collision avoidance, lane reservation, congestion tracking, and automatic deadlock detection and resolution.

## Overview

This system simulates autonomous robots sharing a typed lane network — the kind of environment found in warehouses, factory floors, or logistics hubs. Each lane carries metadata describing its type, safety level, speed limit, and capacity. Robots plan optimal routes, coordinate access to contested lanes, and recover automatically when they deadlock.

---

## Features

- **Typed lane network** — normal, narrow, intersection, and human-zone lanes with independent speed limits, capacities, and safety levels
- **A\* path planning** with congestion-weighted edge costs and lane-type multipliers
- **Lane reservation system** — mutual exclusion for high-risk lanes; capacity-based throttling for corridors
- **Live congestion heatmap** — per-lane occupancy and usage frequency fed back into planning costs
- **Deadlock detection** — DFS cycle detection on the wait-for graph, rebuilt every simulation step
- **Automatic deadlock resolution** — victim robot selected by shortest remaining path, rerouted with congestion avoidance
- **Streamlit dashboard** — interactive network visualisation, animated robot playback, per-robot trajectory detail, and simulation timelines
- **JSON metrics export** — per-scenario results saved for analysis

---

## Project Structure

```
laneawaretrafficcontrol/
│
├── enums.py                        # LaneType and SafetyLevel enumerations
├── data_structures.py              # LaneMetadata, Lane, Robot dataclasses
├── lane_graph.py                   # Adjacency-list graph with directed/undirected lanes
├── heatmap.py                      # Per-lane usage frequency, occupancy, congestion history
├── path_planner.py                 # A* pathfinding with congestion-aware edge costs
├── deadlock_detector.py            # DFS cycle detection and victim-based resolution
├── traffic_controller.py           # Main simulation loop — coordination and metrics
├── scenario_builder.py             # Warehouse and circular deadlock scenario factories
├── visualizer.py                   # Matplotlib visualisations (state, heatmap, trajectories)
├── config.py                       # All tunable constants
├── main.py                         # Entry point — runs both scenarios
│
├── metrics_warehouse_layout.json   # Saved results from the warehouse scenario
├── metrics_circular_deadlock.json  # Saved results from the deadlock scenario
│
└── dashboard.py                    # Streamlit interactive dashboard
```

---

## Installation

**Requirements:** Python 3.8 or higher.

```bash
# Clone or unzip the project
cd laneawaretrafficcontrol

# Install simulation dependencies
pip install numpy matplotlib

# Install dashboard dependencies (optional)
pip install streamlit plotly pandas
```

No additional dependencies beyond the standard library and the packages above.

---

## Usage

### Run the simulation

```bash
cd laneawaretrafficcontrol
python main.py
```

This runs both scenarios sequentially, prints per-step progress to the terminal, displays Matplotlib visualisations, and writes metrics JSON files.

### Run the dashboard only

```bash
# Place dashboard.py alongside the project folder, then:
streamlit run dashboard.py
```

## Scenarios

### Scenario 1 — Warehouse Layout

A 10×10 grid of 100 nodes connected by 180 bidirectional lanes.

Eight robots navigate from corner to corner and across the grid. Expected result: all 8 complete in 19 steps with zero delay and zero deadlocks.

### Scenario 2 — Circular Deadlock

Eight nodes arranged in a circle with directed narrow lanes running clockwise. Every robot immediately claims its first lane, then blocks its neighbour, forming a perfect 8-robot wait cycle at timestep 3.

The deadlock detector identifies the cycle, selects the robot with the shortest remaining path as the victim, releases its reservations, and replans it with congestion avoidance. All 8 robots complete in 24 steps with a total delay of 12.3 seconds.

---

## System Architecture

```
ScenarioBuilder
    └── creates LaneGraph (nodes + typed lanes)
    └── passes to TrafficController

TrafficController
    ├── LaneHeatmap       — passive observer; tracks occupancy and frequency
    ├── PathPlanner       — A* planner; reads heatmap congestion scores
    ├── DeadlockDetector  — DFS cycle detection; calls planner on resolution
    └── Robot[]           — path, trajectory, reservations, wait state

Per simulation step:
    1. _update_congestion()   — heatmap occupancy → lane.congestion_score
    2. _check_deadlocks()     — build wait-for graph → DFS → resolve if cycle found
    3. _update_robot() × N    — reserve lane → check capacity → move → release previous

Feedback loop:
    Robot moves
    → heatmap.update_occupancy() increments lane occupancy
    → _update_congestion() writes new congestion_score to LaneMetadata
    → next robot's compute_path() reads get_effective_speed() via congestion_score
    → planner inflates cost of busy lane → robot routes around it
```

---

## Core Algorithms

### A* Path Planning

Edge cost is estimated traversal time, adjusted for congestion and lane properties:

```
cost = (distance / effective_speed) × (1 + congestion/100) × multipliers
```

Where:

```
effective_speed = max_speed × max(0.2, 1 − congestion/100)
```

| Condition | Multiplier |
|---|---|
| Safety = LOW | 1.5× |
| Lane type = NARROW | 1.3× |
| Lane type = INTERSECTION | 1.2× |
| Lane type = HUMAN_ZONE | 2.0× |

The Euclidean distance heuristic is admissible, guaranteeing the optimal path.

### Congestion Scoring

```
congestion = min(100, occupancy × 30 + usage_frequency × 2)
```

The occupancy term (weight 30) gives an immediate signal when a lane is currently occupied. The frequency term (weight 2) builds up over time to flag structural bottlenecks. History is kept as a rolling window of the last 100 readings.

### Deadlock Detection

A wait-for graph is maintained as a directed graph where an edge from robot A to robot B means A is waiting for a lane currently reserved by B. DFS is run on this graph every step. A back-edge — an edge pointing to a node already on the current call stack — indicates a cycle, which is a deadlock.

### Deadlock Resolution

The robot with the fewest steps remaining in its current path is selected as the victim. Its lane reservations are released, and A* replans its route with `avoid_congested=True`. The other robots in the cycle are unblocked automatically when the victim's lanes are freed.

---


## Dashboard

The Streamlit dashboard (`dashboard.py`) provides four pages:

**Overview** — side-by-side metric cards for both scenarios and a grouped bar chart comparing them. Deadlock events are displayed in expandable cards with robot states at the moment of detection.

**Warehouse Scenario** — four tabs:
- Network: interactive lane graph colour-coded by type, with edge width proportional to usage
- Robot Trajectories: combined path overlay, animated step-by-step playback, per-robot path chart showing time progression, step-by-step position table
- Lane Heatmap: red-intensity overlay showing which lanes were most congested, with a top-15 usage table
- Timeline: area charts for robots completed, robots waiting, and cumulative delay

**Deadlock Scenario** — same structure, with an additional Deadlock Analysis tab showing the detected wait cycle, robot states at detection, and a walkthrough of the five resolution steps.

**Code Structure** — module reference table, data flow diagram, the A* cost formula in LaTeX, and the congestion score formula.

---

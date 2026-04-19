"""
Scenario creation and execution
"""

import json
import numpy as np
from typing import List, Tuple
from lane_graph import LaneGraph
from data_structures import LaneMetadata
from enums import LaneType, SafetyLevel
from traffic_controller import TrafficController
from visualizer import Visualizer


def create_deadlock_scenario() -> LaneGraph:
    """Create a scenario designed to cause deadlock"""
    graph = LaneGraph()

    # Create a circular arrangement with 8 nodes
    num_nodes = 8
    radius = 5.0

    for i in range(num_nodes):
        angle = 2 * np.pi * i / num_nodes
        x = radius * np.cos(angle)
        y = radius * np.sin(angle)
        graph.add_node(i, (x, y))

    # Add directed lanes in a circle
    for i in range(num_nodes):
        next_node = (i + 1) % num_nodes

        metadata = LaneMetadata(
            max_speed=2.0,
            safety_level=SafetyLevel.MEDIUM,
            lane_type=LaneType.NARROW,
            capacity=1
        )

        graph.add_lane(i, next_node, metadata,
                      is_directed=True,
                      requires_reservation=True)

    return graph


def create_warehouse_scenario() -> LaneGraph:
    """Create a realistic warehouse layout"""
    graph = LaneGraph()

    # Create grid layout (10x10)
    for i in range(10):
        for j in range(10):
            node_id = i * 10 + j
            graph.add_node(node_id, (i * 2.0, j * 2.0))

    # Add horizontal lanes
    for i in range(10):
        for j in range(9):
            from_node = i * 10 + j
            to_node = i * 10 + j + 1

            # Determine lane type
            if i in [2, 5, 7]:  # Intersections
                lane_type = LaneType.INTERSECTION
                safety = SafetyLevel.MEDIUM
                capacity = 1
                requires_reservation = True
            elif i == 4:  # Human zone
                lane_type = LaneType.HUMAN_ZONE
                safety = SafetyLevel.LOW
                capacity = 1
                requires_reservation = True
            elif j in [0, 8]:  # Narrow lanes
                lane_type = LaneType.NARROW
                safety = SafetyLevel.MEDIUM
                capacity = 1
                requires_reservation = False
            else:
                lane_type = LaneType.NORMAL
                safety = SafetyLevel.HIGH
                capacity = 2
                requires_reservation = False

            metadata = LaneMetadata(
                max_speed=2.0 if lane_type == LaneType.HUMAN_ZONE else 4.0,
                safety_level=safety,
                lane_type=lane_type,
                capacity=capacity
            )

            graph.add_lane(from_node, to_node, metadata,
                          is_directed=False,
                          requires_reservation=requires_reservation)

    # Add vertical lanes
    for i in range(9):
        for j in range(10):
            from_node = i * 10 + j
            to_node = (i + 1) * 10 + j

            if j in [3, 6]:  # Intersections
                lane_type = LaneType.INTERSECTION
                safety = SafetyLevel.MEDIUM
                capacity = 1
                requires_reservation = True
            elif j == 5:  # Narrow
                lane_type = LaneType.NARROW
                safety = SafetyLevel.MEDIUM
                capacity = 1
                requires_reservation = False
            else:
                lane_type = LaneType.NORMAL
                safety = SafetyLevel.HIGH
                capacity = 2
                requires_reservation = False

            metadata = LaneMetadata(
                max_speed=4.0,
                safety_level=safety,
                lane_type=lane_type,
                capacity=capacity
            )

            graph.add_lane(from_node, to_node, metadata,
                          is_directed=False,
                          requires_reservation=requires_reservation)

    return graph


def run_scenario(scenario_name: str, graph: LaneGraph,
                robot_configs: List[Tuple[int, int]]) -> TrafficController:
    """Run a simulation scenario"""
    print("\n" + "="*70)
    print(f"SCENARIO: {scenario_name}")
    print("="*70)

    # Initialize controller
    print("\n[1/5] Initializing traffic controller...")
    controller = TrafficController(graph, num_robots=len(robot_configs))

    # Add robots
    print("\n[2/5] Adding robots...")
    for i, (start, goal) in enumerate(robot_configs):
        controller.add_robot(i, start, goal)
        print(f"  - Robot {i}: {start} → {goal}")

    # Run simulation
    print("\n[3/5] Running simulation...")
    num_steps = 500

    for step in range(num_steps):
        controller.update()

        if step % 50 == 0:
            completed = controller.metrics['completed']
            waiting = sum(1 for r in controller.robots.values() if r.waiting)
            print(f"  Step {step:3d}: Completed={completed}/{len(robot_configs)}, "
                  f"Waiting={waiting}, "
                  f"Deadlocks={controller.metrics['deadlocks_resolved']}")

        # Stop if all completed
        if controller.metrics['completed'] == len(controller.robots):
            print(f"  All robots completed at step {step}!")
            break

    # Print metrics
    print("\n[4/5] Final Metrics:")
    print(f"  Total Steps: {controller.time_step}")
    print(f"  Robots Completed: {controller.metrics['completed']}/{len(controller.robots)}")
    print(f"  Total Delay: {controller.metrics['total_delay']:.2f}s")
    print(f"  Deadlocks Resolved: {controller.metrics['deadlocks_resolved']}")
    print(f"  Path Replans: {controller.metrics['replans']}")
    print(f"  Collisions Avoided: {controller.metrics['collisions_avoided']}")

    avg_delay = controller.metrics['total_delay'] / max(controller.metrics['completed'], 1)
    print(f"  Average Delay per Robot: {avg_delay:.2f}s")

    # Heatmap analysis
    hotspots = controller.heatmap.get_hotspots(threshold=3)
    print(f"  Congestion Hotspots: {len(hotspots)}")

    # Deadlock analysis
    if controller.deadlock_detector.deadlock_events:
        print(f"\n  Deadlock Events Detected: {len(controller.deadlock_detector.deadlock_events)}")
        for idx, event in enumerate(controller.deadlock_detector.deadlock_events):
            print(f"    Event {idx + 1}: Timestep {event['timestep']}, "
                  f"Cycle: {event['cycle']}")

    # Create visualization
    print("\n[5/5] Creating visualizations...")
    viz = Visualizer(controller)

    print("\nDisplaying: Current State...")
    viz.visualize_current_state()

    print("Displaying: Heatmap...")
    viz.visualize_heatmap()

    print("Displaying: Individual Trajectories...")
    viz.visualize_individual_trajectories()

    print("Displaying: Deadlock Events...")
    viz.visualize_deadlock_events()

    # Export metrics
    metrics_data = {
        'scenario': scenario_name,
        'total_steps': controller.time_step,
        'completed': controller.metrics['completed'],
        'total_robots': len(controller.robots),
        'total_delay': controller.metrics['total_delay'],
        'deadlocks_resolved': controller.metrics['deadlocks_resolved'],
        'replans': controller.metrics['replans'],
        'collisions_avoided': controller.metrics['collisions_avoided'],
        'average_delay': avg_delay,
        'hotspots': len(hotspots),
        'deadlock_events': controller.deadlock_detector.deadlock_events
    }

    filename = f'metrics_{scenario_name.lower().replace(" ", "_")}.json'
    with open(filename, 'w') as f:
        json.dump(metrics_data, f, indent=2)
    print(f"\nSaved metrics: {filename}")

    print("\nScenario complete!")
    return controller
"""
Deadlock detection and resolution
"""

from typing import Dict, List, Set
from collections import defaultdict
from data_structures import Robot
from lane_graph import LaneGraph
from path_planner import PathPlanner


class DeadlockDetector:
    """Detects and resolves deadlocks in the system"""
    
    def __init__(self):
        self.wait_for_graph: Dict[int, Set[int]] = defaultdict(set)
        self.deadlock_events: List[Dict] = []

    def update_waiting(self, robot_id: int, waiting_for_robots: Set[int]):
        """Update the wait-for graph"""
        self.wait_for_graph[robot_id] = waiting_for_robots

    def detect_deadlock(self) -> List[List[int]]:
        """Detect cycles in wait-for graph using DFS"""
        cycles = []
        visited = set()
        rec_stack = set()

        def dfs(node, path):
            visited.add(node)
            rec_stack.add(node)
            path.append(node)

            for neighbor in self.wait_for_graph.get(node, []):
                if neighbor not in visited:
                    if dfs(neighbor, path.copy()):
                        return True
                elif neighbor in rec_stack:
                    # Found cycle
                    cycle_start = path.index(neighbor)
                    cycle = path[cycle_start:]
                    cycles.append(cycle)
                    return True

            rec_stack.remove(node)
            return False

        for node in self.wait_for_graph.keys():
            if node not in visited:
                dfs(node, [])

        return cycles

    def resolve_deadlock(self, cycle: List[int], robots: Dict[int, Robot],
                        graph: LaneGraph, planner: PathPlanner, timestep: int):
        """Resolve deadlock by rerouting lowest priority robot"""
        if not cycle:
            return

        # Record deadlock event
        self.deadlock_events.append({
            'timestep': timestep,
            'cycle': cycle.copy(),
            'robot_states': {rid: robots[rid].current_node for rid in cycle}
        })

        # Choose robot with shortest remaining path to reroute
        victim_id = min(cycle, key=lambda rid: len(robots[rid].path) -
                       robots[rid].path_index)
        victim = robots[victim_id]

        print(f"  [DEADLOCK] Detected at step {timestep}: Cycle = {cycle}")
        print(f"  [DEADLOCK] Resolving by rerouting Robot {victim_id}")

        # Release reservations
        for lane in victim.reserved_lanes:
            lane_obj = graph.get_lane(lane[0], lane[1])
            if lane_obj and lane_obj.reserved_by == victim_id:
                lane_obj.reserved_by = None
        victim.reserved_lanes.clear()

        # Compute alternative path
        new_path = planner.compute_path(victim.current_node,
                                       victim.goal_node,
                                       avoid_congested=True)
        if new_path:
            victim.path = new_path
            victim.path_index = 0
            victim.waiting = False
            victim.waiting_for_lane = None
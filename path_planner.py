"""
Path planning with A* algorithm
"""

import heapq
import numpy as np
from typing import List, Dict
from lane_graph import LaneGraph
from heatmap import LaneHeatmap
from enums import SafetyLevel, LaneType


class PathPlanner:
    """A* path planner with congestion awareness"""
    
    def __init__(self, graph: LaneGraph, heatmap: LaneHeatmap):
        self.graph = graph
        self.heatmap = heatmap

    def compute_path(self, start: int, goal: int,
                     avoid_congested: bool = True) -> List[int]:
        """A* pathfinding with congestion awareness"""
        if start == goal:
            return [start]

        open_set = [(0, start)]
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self._heuristic(start, goal)}

        while open_set:
            _, current = heapq.heappop(open_set)

            if current == goal:
                return self._reconstruct_path(came_from, current)

            for neighbor in self.graph.get_neighbors(current):
                lane = self.graph.get_lane(current, neighbor)
                if not lane:
                    continue

                # Calculate edge cost with congestion penalty
                edge_cost = self._calculate_edge_cost(current, neighbor,
                                                     avoid_congested)
                tentative_g = g_score[current] + edge_cost

                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f = tentative_g + self._heuristic(neighbor, goal)
                    f_score[neighbor] = f
                    heapq.heappush(open_set, (f, neighbor))

        return []  # No path found

    def _heuristic(self, node1: int, node2: int) -> float:
        """Euclidean distance heuristic"""
        pos1 = self.graph.nodes[node1]
        pos2 = self.graph.nodes[node2]
        return np.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)

    def _calculate_edge_cost(self, from_node: int, to_node: int,
                            avoid_congested: bool) -> float:
        """Calculate edge traversal cost"""
        lane = self.graph.get_lane(from_node, to_node)
        distance = self._heuristic(from_node, to_node)

        # Base cost is distance / speed
        base_cost = distance / lane.metadata.get_effective_speed()

        # Add congestion penalty
        if avoid_congested:
            congestion = self.heatmap.get_congestion_score(from_node, to_node)
            base_cost *= (1.0 + congestion / 100.0)

        # Add safety penalty for dangerous lanes
        if lane.metadata.safety_level == SafetyLevel.LOW:
            base_cost *= 1.5

        # Add penalty for narrow lanes and intersections
        if lane.metadata.lane_type == LaneType.NARROW:
            base_cost *= 1.3
        elif lane.metadata.lane_type == LaneType.INTERSECTION:
            base_cost *= 1.2
        elif lane.metadata.lane_type == LaneType.HUMAN_ZONE:
            base_cost *= 2.0

        return base_cost

    def _reconstruct_path(self, came_from: Dict[int, int],
                         current: int) -> List[int]:
        """Reconstruct path from came_from dictionary"""
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path
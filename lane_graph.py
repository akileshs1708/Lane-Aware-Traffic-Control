"""
Lane graph representation
"""

from typing import Dict, List, Tuple, Optional
from collections import defaultdict
from data_structures import Lane, LaneMetadata


class LaneGraph:
    """Graph structure representing the lane network"""
    
    def __init__(self):
        self.nodes: Dict[int, Tuple[float, float]] = {}
        self.lanes: Dict[Tuple[int, int], Lane] = {}
        self.adjacency: Dict[int, List[int]] = defaultdict(list)

    def add_node(self, node_id: int, position: Tuple[float, float]):
        """Add a node to the graph"""
        self.nodes[node_id] = position

    def add_lane(self, from_node: int, to_node: int, metadata: LaneMetadata,
                 is_directed: bool = True, requires_reservation: bool = False):
        """Add a lane to the graph"""
        lane = Lane(from_node, to_node, metadata, is_directed, requires_reservation)
        self.lanes[(from_node, to_node)] = lane
        self.adjacency[from_node].append(to_node)

        if not is_directed:
            reverse_metadata = LaneMetadata(
                metadata.max_speed, metadata.safety_level, metadata.lane_type,
                metadata.congestion_score, metadata.usage_count,
                metadata.current_occupancy, metadata.capacity
            )
            reverse_lane = Lane(to_node, from_node, reverse_metadata,
                              is_directed, requires_reservation)
            self.lanes[(to_node, from_node)] = reverse_lane
            self.adjacency[to_node].append(from_node)

    def get_lane(self, from_node: int, to_node: int) -> Optional[Lane]:
        """Get a lane by its endpoints"""
        return self.lanes.get((from_node, to_node))

    def get_neighbors(self, node: int) -> List[int]:
        """Get neighboring nodes"""
        return self.adjacency.get(node, [])
"""
Lane usage heatmap tracking
"""

from typing import Dict, List, Tuple
from collections import defaultdict


class LaneHeatmap:
    """Tracks lane usage and congestion over time"""
    
    def __init__(self):
        self.usage_frequency: Dict[Tuple[int, int], int] = defaultdict(int)
        self.real_time_occupancy: Dict[Tuple[int, int], int] = defaultdict(int)
        self.congestion_history: Dict[Tuple[int, int], List[float]] = defaultdict(list)

    def record_usage(self, from_node: int, to_node: int):
        """Record usage of a lane"""
        self.usage_frequency[(from_node, to_node)] += 1

    def update_occupancy(self, from_node: int, to_node: int, delta: int):
        """Update current occupancy of a lane"""
        self.real_time_occupancy[(from_node, to_node)] = max(
            0, self.real_time_occupancy[(from_node, to_node)] + delta
        )

    def record_congestion(self, from_node: int, to_node: int, score: float):
        """Record congestion score for a lane"""
        history = self.congestion_history[(from_node, to_node)]
        history.append(score)
        if len(history) > 100:  # Keep last 100 records
            history.pop(0)

    def get_hotspots(self, threshold: int = 5) -> List[Tuple[int, int]]:
        """Get lanes with high usage (hotspots)"""
        return [lane for lane, count in self.usage_frequency.items()
                if count > threshold]

    def get_congestion_score(self, from_node: int, to_node: int) -> float:
        """Calculate congestion score for a lane"""
        occupancy = self.real_time_occupancy[(from_node, to_node)]
        frequency = self.usage_frequency[(from_node, to_node)]
        return min(100.0, occupancy * 30 + frequency * 2)
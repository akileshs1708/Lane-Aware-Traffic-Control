"""
Core data structures for robots and lanes
"""

from dataclasses import dataclass, field
from typing import List, Set, Tuple, Optional
from enums import LaneType, SafetyLevel


@dataclass
class LaneMetadata:
    """Metadata associated with a lane"""
    max_speed: float
    safety_level: SafetyLevel
    lane_type: LaneType
    congestion_score: float = 0.0
    usage_count: int = 0
    current_occupancy: int = 0
    capacity: int = 1

    def get_effective_speed(self) -> float:
        """Calculate effective speed based on congestion"""
        congestion_factor = max(0.2, 1.0 - (self.congestion_score / 100.0))
        return self.max_speed * congestion_factor


@dataclass
class Lane:
    """Represents a lane between two nodes"""
    from_node: int
    to_node: int
    metadata: LaneMetadata
    is_directed: bool = True
    requires_reservation: bool = False
    reserved_by: Optional[int] = None

    def can_enter(self, robot_id: int) -> bool:
        """Check if robot can enter this lane"""
        if self.requires_reservation:
            return self.reserved_by is None or self.reserved_by == robot_id
        return self.metadata.current_occupancy < self.metadata.capacity


@dataclass
class Robot:
    """Represents a robot in the system"""
    id: int
    position: Tuple[float, float]
    current_node: int
    goal_node: int
    speed: float = 1.0
    path: List[int] = field(default_factory=list)
    path_index: int = 0
    waiting: bool = False
    waiting_for_lane: Optional[Tuple[int, int]] = None
    reserved_lanes: Set[Tuple[int, int]] = field(default_factory=set)
    state: str = "active"
    trajectory: List[Tuple[float, float, int]] = field(default_factory=list)  # (x, y, timestep)
    wait_count: int = 0
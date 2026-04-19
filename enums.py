"""
Enumerations for the traffic control system
"""

from enum import Enum


class LaneType(Enum):
    """Types of lanes in the system"""
    NORMAL = "normal"
    NARROW = "narrow"
    INTERSECTION = "intersection"
    HUMAN_ZONE = "human_zone"


class SafetyLevel(Enum):
    """Safety levels for lanes"""
    HIGH = 3
    MEDIUM = 2
    LOW = 1
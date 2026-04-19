"""
Main traffic control logic
"""

from typing import Dict, Tuple
from collections import defaultdict
from lane_graph import LaneGraph
from heatmap import LaneHeatmap
from path_planner import PathPlanner
from deadlock_detector import DeadlockDetector
from data_structures import Robot


class TrafficController:
    """Main controller for multi-robot traffic management"""
    
    def __init__(self, graph: LaneGraph, num_robots: int = 8):
        self.graph = graph
        self.heatmap = LaneHeatmap()
        self.planner = PathPlanner(graph, self.heatmap)
        self.deadlock_detector = DeadlockDetector()
        self.robots: Dict[int, Robot] = {}
        self.metrics = {
            'completed': 0,
            'total_delay': 0.0,
            'deadlocks_resolved': 0,
            'replans': 0,
            'collisions_avoided': 0
        }
        self.time_step = 0

    def add_robot(self, robot_id: int, start_node: int, goal_node: int):
        """Add a robot to the system"""
        position = self.graph.nodes[start_node]
        path = self.planner.compute_path(start_node, goal_node)

        robot = Robot(
            id=robot_id,
            position=position,
            current_node=start_node,
            goal_node=goal_node,
            path=path
        )
        robot.trajectory.append((position[0], position[1], 0))
        self.robots[robot_id] = robot

    def update(self, dt: float = 0.1):
        """Main update loop"""
        self.time_step += 1

        # Update congestion scores
        self._update_congestion()

        # Check for deadlocks
        self._check_deadlocks()

        # Update each robot
        for robot in self.robots.values():
            if robot.state == "completed":
                continue

            self._update_robot(robot, dt)

            # Record trajectory
            robot.trajectory.append((robot.position[0], robot.position[1], self.time_step))

    def _update_congestion(self):
        """Update congestion scores for all lanes"""
        for lane_key, lane in self.graph.lanes.items():
            occupancy = self.heatmap.real_time_occupancy[lane_key]
            congestion = min(100.0, (occupancy / lane.metadata.capacity) * 100)
            lane.metadata.congestion_score = congestion
            self.heatmap.record_congestion(lane_key[0], lane_key[1], congestion)

    def _check_deadlocks(self):
        """Detect and resolve deadlocks"""
        # Build wait-for graph
        for robot in self.robots.values():
            if robot.waiting and robot.waiting_for_lane:
                lane = self.graph.get_lane(robot.waiting_for_lane[0],
                                          robot.waiting_for_lane[1])
                if lane and lane.reserved_by is not None:
                    self.deadlock_detector.update_waiting(
                        robot.id, {lane.reserved_by}
                    )
                else:
                    self.deadlock_detector.update_waiting(robot.id, set())
            else:
                self.deadlock_detector.update_waiting(robot.id, set())

        # Detect cycles
        cycles = self.deadlock_detector.detect_deadlock()

        # Resolve each deadlock
        for cycle in cycles:
            self.deadlock_detector.resolve_deadlock(
                cycle, self.robots, self.graph, self.planner, self.time_step
            )
            self.metrics['deadlocks_resolved'] += 1

    def _update_robot(self, robot: Robot, dt: float):
        """Update single robot state"""
        # Check if reached goal
        if robot.current_node == robot.goal_node:
            robot.state = "completed"
            self.metrics['completed'] += 1
            # Release all reservations
            for lane in robot.reserved_lanes:
                lane_obj = self.graph.get_lane(lane[0], lane[1])
                if lane_obj:
                    lane_obj.reserved_by = None
                    self.heatmap.update_occupancy(lane[0], lane[1], -1)
            robot.reserved_lanes.clear()
            return

        # Check if path is valid
        if robot.path_index >= len(robot.path) - 1:
            # Need to replan
            new_path = self.planner.compute_path(robot.current_node,
                                                robot.goal_node)
            if new_path:
                robot.path = new_path
                robot.path_index = 0
                self.metrics['replans'] += 1
            else:
                robot.waiting = True
                return

        # Get next node
        next_node = robot.path[robot.path_index + 1]
        lane = self.graph.get_lane(robot.current_node, next_node)

        if not lane:
            # Invalid lane, replan
            robot.path = self.planner.compute_path(robot.current_node,
                                                  robot.goal_node)
            robot.path_index = 0
            self.metrics['replans'] += 1
            return

        # Try to reserve lane
        if lane.requires_reservation:
            if not lane.can_enter(robot.id):
                robot.waiting = True
                robot.waiting_for_lane = (robot.current_node, next_node)
                robot.wait_count += 1
                self.metrics['total_delay'] += dt

                # Check if should replan (only after significant waiting)
                if robot.wait_count > 30:  # Wait longer before replanning
                    new_path = self.planner.compute_path(robot.current_node,
                                                        robot.goal_node)
                    if new_path != robot.path:
                        robot.path = new_path
                        robot.path_index = 0
                        robot.waiting = False
                        robot.waiting_for_lane = None
                        robot.wait_count = 0
                        self.metrics['replans'] += 1
                return
            else:
                # Reserve lane
                lane.reserved_by = robot.id
                robot.reserved_lanes.add((robot.current_node, next_node))

        # Check capacity
        if lane.metadata.current_occupancy >= lane.metadata.capacity:
            robot.waiting = True
            robot.waiting_for_lane = (robot.current_node, next_node)
            robot.wait_count += 1
            self.metrics['total_delay'] += dt
            self.metrics['collisions_avoided'] += 1
            return

        # Move robot
        robot.waiting = False
        robot.waiting_for_lane = None
        robot.wait_count = 0

        # Update lane occupancy
        self.heatmap.update_occupancy(robot.current_node, next_node, 1)
        self.heatmap.record_usage(robot.current_node, next_node)
        lane.metadata.usage_count += 1

        # Release previous lane
        if robot.path_index > 0:
            prev_node = robot.path[robot.path_index - 1]
            prev_lane = self.graph.get_lane(prev_node, robot.current_node)
            if prev_lane:
                self.heatmap.update_occupancy(prev_node, robot.current_node, -1)
                if prev_lane.reserved_by == robot.id:
                    prev_lane.reserved_by = None
                    robot.reserved_lanes.discard((prev_node, robot.current_node))

        # Update position
        robot.current_node = next_node
        robot.position = self.graph.nodes[next_node]
        robot.path_index += 1

        # Update speed based on lane
        robot.speed = lane.metadata.get_effective_speed()
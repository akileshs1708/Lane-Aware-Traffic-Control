"""
Visualization functions
"""

import numpy as np
import matplotlib.pyplot as plt
from traffic_controller import TrafficController
from enums import LaneType


class Visualizer:
    """Handles all visualization tasks"""
    
    def __init__(self, controller: TrafficController):
        self.controller = controller

    def visualize_heatmap(self):
        """Visualize lane usage heatmap"""
        fig, ax = plt.subplots(figsize=(12, 10))

        # Draw lanes with heat colors
        max_usage = max(self.controller.heatmap.usage_frequency.values(),
                       default=1)

        for (from_node, to_node), usage in \
                self.controller.heatmap.usage_frequency.items():
            pos1 = self.controller.graph.nodes[from_node]
            pos2 = self.controller.graph.nodes[to_node]

            intensity = usage / max_usage
            color = plt.cm.hot(intensity)

            ax.plot([pos1[0], pos2[0]], [pos1[1], pos2[1]],
                   color=color, linewidth=3, alpha=0.7)

        # Draw nodes
        for node_id, pos in self.controller.graph.nodes.items():
            ax.plot(pos[0], pos[1], 'ko', markersize=10)
            ax.text(pos[0], pos[1], str(node_id),
                   fontsize=7, ha='center', va='center',
                   bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))

        # Add colorbar
        sm = plt.cm.ScalarMappable(cmap=plt.cm.hot,
                                   norm=plt.Normalize(vmin=0, vmax=max_usage))
        sm.set_array([])
        cbar = plt.colorbar(sm, ax=ax)
        cbar.set_label('Usage Frequency', rotation=270, labelpad=20)

        ax.set_title('Lane Usage Heatmap', fontsize=14, fontweight='bold')
        ax.set_xlabel('X', fontsize=12)
        ax.set_ylabel('Y', fontsize=12)
        ax.grid(True, alpha=0.3)

        plt.tight_layout()
        plt.show()

    def visualize_current_state(self):
        """Visualize current robot positions and lanes"""
        fig, ax = plt.subplots(figsize=(12, 10))

        # Draw lanes
        for (from_node, to_node), lane in self.controller.graph.lanes.items():
            pos1 = self.controller.graph.nodes[from_node]
            pos2 = self.controller.graph.nodes[to_node]

            # Color by lane type
            if lane.metadata.lane_type == LaneType.INTERSECTION:
                color = 'orange'
            elif lane.metadata.lane_type == LaneType.NARROW:
                color = 'red'
            elif lane.metadata.lane_type == LaneType.HUMAN_ZONE:
                color = 'purple'
            else:
                color = 'gray'

            alpha = 0.3 + (lane.metadata.congestion_score / 100.0) * 0.7
            ax.plot([pos1[0], pos2[0]], [pos1[1], pos2[1]],
                   color=color, linewidth=2, alpha=alpha)

            # Draw arrow for directed lanes
            if lane.is_directed:
                mid_x = (pos1[0] + pos2[0]) / 2
                mid_y = (pos1[1] + pos2[1]) / 2
                dx = pos2[0] - pos1[0]
                dy = pos2[1] - pos1[1]
                ax.arrow(mid_x - dx*0.1, mid_y - dy*0.1,
                        dx*0.2, dy*0.2, head_width=0.3,
                        head_length=0.2, fc=color, ec=color, alpha=0.5)

        # Draw nodes
        for node_id, pos in self.controller.graph.nodes.items():
            ax.plot(pos[0], pos[1], 'ko', markersize=8)

        # Draw robots
        colors = plt.cm.tab10(np.linspace(0, 1, len(self.controller.robots)))
        for robot, color in zip(self.controller.robots.values(), colors):
            marker = 'X' if robot.waiting else 'o'
            size = 200 if robot.state == "completed" else 150
            edge_color = 'red' if robot.waiting else 'black'
            ax.scatter(robot.position[0], robot.position[1],
                      c=[color], s=size, marker=marker,
                      edgecolors=edge_color, linewidths=2, zorder=5)
            ax.text(robot.position[0] + 0.6, robot.position[1] + 0.6,
                   f'R{robot.id}', fontsize=9, fontweight='bold')

            # Draw path
            if robot.path and robot.state != "completed":
                path_x = [self.controller.graph.nodes[n][0]
                         for n in robot.path[robot.path_index:]]
                path_y = [self.controller.graph.nodes[n][1]
                         for n in robot.path[robot.path_index:]]
                ax.plot(path_x, path_y, '--', color=color,
                       alpha=0.4, linewidth=2)

        ax.set_title(f'Robot Positions (Step {self.controller.time_step})',
                    fontsize=14, fontweight='bold')
        ax.set_xlabel('X', fontsize=12)
        ax.set_ylabel('Y', fontsize=12)
        ax.grid(True, alpha=0.3)

        # Create legend
        legend_elements = [
            plt.Line2D([0], [0], color='gray', lw=2, label='Normal Lane'),
            plt.Line2D([0], [0], color='orange', lw=2, label='Intersection'),
            plt.Line2D([0], [0], color='red', lw=2, label='Narrow Lane'),
            plt.Line2D([0], [0], color='purple', lw=2, label='Human Zone')
        ]
        ax.legend(handles=legend_elements, loc='upper left', fontsize=10)

        plt.tight_layout()
        plt.show()

    def visualize_individual_trajectories(self):
        """Create separate trajectory plot for each robot"""
        for robot_id, robot in self.controller.robots.items():
            fig, ax = plt.subplots(figsize=(10, 8))

            # Draw all lanes (faded)
            for (from_node, to_node), lane in self.controller.graph.lanes.items():
                pos1 = self.controller.graph.nodes[from_node]
                pos2 = self.controller.graph.nodes[to_node]
                ax.plot([pos1[0], pos2[0]], [pos1[1], pos2[1]],
                       color='lightgray', linewidth=1, alpha=0.3)

            # Draw all nodes (small)
            for node_id, pos in self.controller.graph.nodes.items():
                ax.plot(pos[0], pos[1], 'o', color='lightgray', markersize=4)

            # Highlight start and goal
            start_pos = self.controller.graph.nodes[robot.path[0]]
            goal_pos = self.controller.graph.nodes[robot.goal_node]
            ax.plot(start_pos[0], start_pos[1], 'go', markersize=15,
                   label='Start', zorder=5)
            ax.plot(goal_pos[0], goal_pos[1], 'r*', markersize=20,
                   label='Goal', zorder=5)

            # Draw trajectory
            if len(robot.trajectory) > 1:
                traj_x = [pos[0] for pos in robot.trajectory]
                traj_y = [pos[1] for pos in robot.trajectory]

                # Color gradient based on time
                num_points = len(traj_x)
                colors_grad = plt.cm.viridis(np.linspace(0, 1, num_points))

                for i in range(num_points - 1):
                    ax.plot(traj_x[i:i+2], traj_y[i:i+2],
                           color=colors_grad[i], linewidth=2, alpha=0.8)

                # Mark waiting points
                wait_points_x = []
                wait_points_y = []
                for i in range(1, len(robot.trajectory)):
                    if robot.trajectory[i][0] == robot.trajectory[i-1][0] and \
                       robot.trajectory[i][1] == robot.trajectory[i-1][1]:
                        wait_points_x.append(robot.trajectory[i][0])
                        wait_points_y.append(robot.trajectory[i][1])

                if wait_points_x:
                    ax.scatter(wait_points_x, wait_points_y, c='red', s=50,
                             marker='x', label='Wait Points', zorder=6)

            # Add path nodes
            path_x = [self.controller.graph.nodes[n][0] for n in robot.path]
            path_y = [self.controller.graph.nodes[n][1] for n in robot.path]
            ax.plot(path_x, path_y, 'b--', linewidth=1, alpha=0.5,
                   label='Planned Path')

            ax.set_title(f'Robot {robot_id} Trajectory\n'
                        f'Start: Node {robot.path[0]}, Goal: Node {robot.goal_node}\n'
                        f'Status: {robot.state.upper()}',
                        fontsize=12, fontweight='bold')
            ax.set_xlabel('X', fontsize=11)
            ax.set_ylabel('Y', fontsize=11)
            ax.grid(True, alpha=0.3)
            ax.legend(loc='best', fontsize=10)
            ax.set_aspect('equal')

            plt.tight_layout()
            plt.show()

    def visualize_deadlock_events(self):
        """Visualize deadlock events"""
        if not self.controller.deadlock_detector.deadlock_events:
            print("\nNo deadlocks detected to visualize.")
            return

        for idx, event in enumerate(self.controller.deadlock_detector.deadlock_events):
            fig, ax = plt.subplots(figsize=(12, 10))

            # Draw all lanes
            for (from_node, to_node), lane in self.controller.graph.lanes.items():
                pos1 = self.controller.graph.nodes[from_node]
                pos2 = self.controller.graph.nodes[to_node]
                ax.plot([pos1[0], pos2[0]], [pos1[1], pos2[1]],
                       color='lightgray', linewidth=1, alpha=0.5)

            # Draw all nodes
            for node_id, pos in self.controller.graph.nodes.items():
                ax.plot(pos[0], pos[1], 'o', color='gray', markersize=6)
                ax.text(pos[0], pos[1], str(node_id),
                       fontsize=7, ha='center', va='center',
                       bbox=dict(boxstyle='round', facecolor='white', alpha=0.7))

            # Highlight robots in deadlock
            cycle = event['cycle']
            robot_states = event['robot_states']

            colors = plt.cm.rainbow(np.linspace(0, 1, len(cycle)))

            for i, (robot_id, color) in enumerate(zip(cycle, colors)):
                node = robot_states[robot_id]
                pos = self.controller.graph.nodes[node]

                ax.scatter(pos[0], pos[1], c=[color], s=300,
                         marker='o', edgecolors='red', linewidths=3, zorder=5)
                ax.text(pos[0] + 0.7, pos[1] + 0.7, f'R{robot_id}',
                       fontsize=11, fontweight='bold', color='red')

                # Draw wait-for relationship
                next_robot_id = cycle[(i + 1) % len(cycle)]
                next_node = robot_states[next_robot_id]
                next_pos = self.controller.graph.nodes[next_node]

                ax.annotate('', xy=next_pos, xytext=pos,
                           arrowprops=dict(arrowstyle='->', color='red',
                                         lw=2, alpha=0.7))

            ax.set_title(f'Deadlock Event #{idx + 1} at Timestep {event["timestep"]}\n'
                        f'Cycle: {" → ".join(map(str, cycle))} → {cycle[0]}',
                        fontsize=13, fontweight='bold')
            ax.set_xlabel('X', fontsize=12)
            ax.set_ylabel('Y', fontsize=12)
            ax.grid(True, alpha=0.3)

            plt.tight_layout()
            plt.show()
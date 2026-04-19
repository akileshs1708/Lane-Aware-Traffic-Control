"""
Main entry point for the Lane-Aware Multi-Robot Traffic Control System
"""

from scenario_builder import (
    create_warehouse_scenario,
    create_deadlock_scenario,
    run_scenario
)


def main():
    """Main execution function"""
    print("="*70)
    print("LANE-AWARE MULTI-ROBOT TRAFFIC CONTROL SYSTEM".center(70))
    print("="*70)

    # Scenario 1: Warehouse
    print("\n\n")
    print("SCENARIO 1: WAREHOUSE LAYOUT (Normal Operations)".center(70))
    print("-"*70)

    warehouse_graph = create_warehouse_scenario()

    # Diverse robot configurations
    warehouse_configs = [
        (0, 99),   # Corner to corner
        (9, 90),   # Corner to corner
        (45, 54),  # Center area
        (23, 76),  # Cross paths
        (5, 94),   # Long distance
        (88, 11),  # Long distance reverse
        (34, 65),  # Medium distance
        (72, 27),  # Medium distance
    ]

    controller_warehouse = run_scenario("Warehouse Layout",
                                       warehouse_graph,
                                       warehouse_configs)

    # Scenario 2: Circular Deadlock
    print("\n\n")
    print("SCENARIO 2: CIRCULAR DEADLOCK (8 Robots in Circle)".center(70))
    print("-"*70)

    deadlock_graph = create_deadlock_scenario()

    # Each robot starts at node i and wants to go to node (i+4) % 8
    deadlock_configs = [
        (0, 4), (1, 5), (2, 6), (3, 7),
        (4, 0), (5, 1), (6, 2), (7, 3)
    ]

    controller_deadlock = run_scenario("Circular Deadlock",
                                      deadlock_graph,
                                      deadlock_configs)

    # Final summary
    print("\n\n")
    print("="*70)
    print("ALL SCENARIOS COMPLETED!".center(70))
    print("="*70)
    print("\n✓ All visualizations displayed!")
    print("✓ Metrics saved to JSON files!")
    print("\nThank you for using the Lane-Aware Traffic Control System!")


if __name__ == "__main__":
    main()
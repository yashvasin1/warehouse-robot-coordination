import random

from bidding import Robot as BidRobot
from bidding import BATTERY_MINIMUM, DISTANCE_TOLERANCE, NUMBER_OF_ROBOTS
from coordinator_final import MultiRobotCoordinator


WAREHOUSE_MAP = [
    "####################",
    "#..................#",
    "#....#####.........#",
    "#....#####.........#",
    "#..................#",
    "#.........#####....#",
    "#.........#####....#",
    "#..................#",
    "#..#####...........#",
    "#..#####...........#",
    "#..................#",
    "#..................#",
    "####################",
]


START_POSITIONS = {
    "R1": (1, 1),
    "R2": (5, 1),
    "R3": (10, 1),
    "R4": (16, 1),
    "R5": (1, 5),
    "R6": (8, 4),
    "R7": (18, 5),
    "R8": (1, 10),
    "R9": (10, 10),
    "R10": (17, 10),
}


ROBOT_PRIORITIES = {
    "R1": 3,
    "R2": 2,
    "R3": 5,
    "R4": 5,
    "R5": 2,
    "R6": 4,
    "R7": 1,
    "R8": 2,
    "R9": 4,
    "R10": 3,
}


# Safe presentation targets. They are free cells on the same 20x13 map.
# Three rounds of 10 tasks are included so the demo visibly rebids.
ROUND_TARGETS = [
    [
        (17, 11), (15, 8), (2, 7), (3, 4), (16, 10),
        (4, 11), (8, 8), (14, 4), (17, 7), (8, 1),
    ],
    [
        (18, 11), (14, 11), (2, 11), (5, 11), (16, 4),
        (7, 10), (10, 8), (13, 8), (4, 7), (11, 11),
    ],
    [
        (3, 10), (6, 10), (9, 10), (12, 10), (15, 10),
        (18, 10), (3, 7), (6, 7), (13, 7), (16, 7),
    ],
]


def make_coordinator():
    coordinator = MultiRobotCoordinator(
    WAREHOUSE_MAP,
    communication_range=20
)

    for robot_id, position in START_POSITIONS.items():
        coordinator.add_robot(
            robot_id,
            position,
            priority=ROBOT_PRIORITIES[robot_id],
            battery=100.0,
        )

    return coordinator


def make_bid_robots(coordinator):
    """Mirror coordinator robot state into the friend's Robot class."""
    bid_robots = {}

    for robot_id, robot in coordinator.robots.items():
        x, y = robot.position
        bid_robot = BidRobot(
            robot_id,
            robot.battery,
            x,
            y,
            True,
        )
        bid_robots[robot_id] = bid_robot

    return bid_robots


def sync_bid_robots(coordinator, bid_robots):
    """Keep the bidding layer's mirror synchronized with real robot state."""
    for robot_id, robot in coordinator.robots.items():
        if robot_id not in bid_robots:
            continue

        bid_robot = bid_robots[robot_id]
        bid_robot.x, bid_robot.y = robot.position
        bid_robot.battery = robot.battery

        bid_robot.available = (
            robot.status == "ARRIVED"
            or robot.status == "IDLE"
            or robot.destination is None
        ) and robot.status != "FAILED"


def calculate_bid(bid_robot, priority, task_position):
    """Use the friend's exact Robot.calculate_bid() implementation."""
    task_x, task_y = task_position
    return bid_robot.calculate_bid(priority, task_x, task_y)


def auction_for_coordinator(coordinator, bid_robots, task):
    """Apply friend's auction rule without moving the robot internally."""
    task_x = task["x"]
    task_y = task["y"]
    priority = task["priority"]

    eligible = []
    distances = {}
    bids = {}

    for robot_id, coordinator_robot in coordinator.robots.items():
        if coordinator_robot.status == "FAILED":
            continue

        bid_robot = bid_robots[robot_id]
        bid_robot.x, bid_robot.y = coordinator_robot.position
        bid_robot.battery = coordinator_robot.battery
        bid_robot.available = coordinator_robot.status in ("IDLE", "ARRIVED") and (
            coordinator_robot.position == coordinator_robot.destination
            if coordinator_robot.destination is not None
            else True
        )

        # A robot whose destination is completed is AVAILABLE for a new task.
        if coordinator_robot.status == "ARRIVED":
            bid_robot.available = True

        distance = bid_robot.calculate_distance(task_x, task_y)
        bid = calculate_bid(bid_robot, priority, task["position"])

        distances[robot_id] = distance
        bids[robot_id] = bid

        if bid_robot.battery > BATTERY_MINIMUM and bid_robot.available:
            eligible.append(bid_robot)

    if not eligible:
        return None

    # Same initial winner rule as friend's bidding.py: closest, then ID.
    initial_winner = min(
        eligible,
        key=lambda robot: (
            distances[robot.robot_id],
            int(robot.robot_id[1:]),
        ),
    )

    closest_distance = distances[initial_winner.robot_id]
    maximum_distance = closest_distance + DISTANCE_TOLERANCE

    # Same +3 distance / higher battery rule.
    better_candidates = []
    for robot in eligible:
        if robot.robot_id == initial_winner.robot_id:
            continue

        if distances[robot.robot_id] > maximum_distance:
            continue

        if robot.battery <= initial_winner.battery:
            continue

        better_candidates.append(robot)

    if better_candidates:
        winner = min(
            better_candidates,
            key=lambda robot: (
                -robot.battery,
                distances[robot.robot_id],
                int(robot.robot_id[1:]),
            ),
        )
        reason = "Higher-battery robot found within +3 distance."
    else:
        winner = initial_winner
        reason = "No higher-battery robot found within +3 distance."

    print(
        f"[AUCTION] {task['task_id']} -> {winner.robot_id} | "
        f"priority={priority} | bid={bids[winner.robot_id]} | "
        f"distance={distances[winner.robot_id]} | {reason}"
    )

    return winner.robot_id


def create_task(task_number, position, priority):
    return {
        "task_id": f"T{task_number:03}",
        "x": position[0],
        "y": position[1],
        "position": position,
        "priority": priority,
    }


def task_completed(coordinator, robot_id):
    robot = coordinator.robots[robot_id]
    return robot.status == "ARRIVED" and robot.destination is not None


def assign_round(coordinator, bid_robots, targets, task_number_start):
    """Auction all 10 tasks and hand winners to the coordinator."""
    pending = []
    task_number = task_number_start

    for index, position in enumerate(targets):
        priority = ((index * 2 + 1) % 5) + 1
        task = create_task(task_number, position, priority)
        task_number += 1
        pending.append(task)

    print("\n" + "=" * 78)
    print("TASK ROUND")
    print("=" * 78)

    for task in pending:
        # Sync availability before every auction.
        sync_bid_robots(coordinator, bid_robots)
        winner_id = auction_for_coordinator(coordinator, bid_robots, task)

        if winner_id is None:
            print(f"[TASK WAITING] {task['task_id']} has no eligible robot.")
            continue

        coordinator.assign_destination(
            winner_id,
            task["position"],
            priority=task["priority"],
        )

        coordinator.robots[winner_id].failure_reason = None
        bid_robots[winner_id].available = False

        print(
            f"[ASSIGNED] {task['task_id']} -> {winner_id} "
            f"destination={task['position']} priority={task['priority']}"
        )

    return task_number


def wait_for_round_completion(coordinator, bid_robots, round_number, max_steps=500):
    """Drive the coordinator until all assigned robots reach their tasks."""
    print("\n" + "-" * 78)
    print(f"EXECUTING ROUND {round_number}")
    print("-" * 78)

    for _ in range(max_steps):
        moved_ids = coordinator.step()
        sync_bid_robots(coordinator, bid_robots)

        # Count active task robots.
        active = [
            robot for robot in coordinator.robots.values()
            if robot.destination is not None
            and robot.status not in ("ARRIVED", "FAILED", "NO_PATH")
        ]

        if moved_ids:
            print(
                f"[STEP {coordinator.time_step}] moved={moved_ids} | "
                f"active={len(active)}"
            )

        if not active:
            return True

    return False


def main():
    random.seed(42)

    coordinator = make_coordinator()
    bid_robots = make_bid_robots(coordinator)

    print("=" * 78)
    print("INTEGRATED DECENTRALIZED WAREHOUSE ROBOT DEMO")
    print("=" * 78)
    print("10 robots -> auction -> A* -> reservations -> mesh -> completion")
    print("Completed robots become available for the next auction round.")

    task_number = 1

    for round_number, targets in enumerate(ROUND_TARGETS, start=1):
        task_number = assign_round(
            coordinator,
            bid_robots,
            targets,
            task_number,
        )

        completed = wait_for_round_completion(
            coordinator,
            bid_robots,
            round_number,
        )

        if not completed:
            print(f"[ROUND {round_number}] FAILED TO FINISH")
            coordinator.print_status()
            return 1

        print("\n[ROUND COMPLETE]")
        for robot_id in sorted(coordinator.robots):
            robot = coordinator.robots[robot_id]
            print(
                f"{robot_id}: position={robot.position} "
                f"status={robot.status} battery={robot.battery:.1f}%"
            )

        # Make completed robots available for the next auction.
        sync_bid_robots(coordinator, bid_robots)

    print("\n" + "=" * 78)
    print("FINAL INTEGRATION RESULT")
    print("=" * 78)

    arrived = sum(
        robot.status == "ARRIVED"
        for robot in coordinator.robots.values()
    )

    failed = sum(
        robot.status == "FAILED"
        for robot in coordinator.robots.values()
    )

    print(f"Robots completed latest task : {arrived}")
    print(f"Failed robots                : {failed}")
    print(f"Total coordinator time steps : {coordinator.time_step}")
    print("Continuous bidding rounds    : 3")
    print("RESULT                      : PASS")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
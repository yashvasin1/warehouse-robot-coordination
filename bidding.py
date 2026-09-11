import random
import time


# ============================================================
# SETTINGS
# ============================================================

BATTERY_START = 100
BATTERY_MINIMUM = 15

# Battery consumed per distance unit
BATTERY_PER_DISTANCE = 0.1

# Number of robots
NUMBER_OF_ROBOTS = 10

# Number of tasks given immediately at the beginning
INITIAL_TASKS = 10

# Time between new tasks
TASK_ARRIVAL_INTERVAL = 3

# How long a robot takes to complete a task
WORK_TIME = 5

# Distance tolerance
DISTANCE_TOLERANCE = 3


# ============================================================
# ROBOT CLASS
# ============================================================

class Robot:

    def __init__(
        self,
        robot_id,
        battery,
        x,
        y,
        available=True
    ):

        self.robot_id = robot_id
        self.battery = battery
        self.x = x
        self.y = y
        self.available = available

        # Current task information
        self.current_task = None
        self.busy_until = 0

        # Statistics
        self.completed_tasks = 0

    # --------------------------------------------------------
    # Calculate distance from robot to task
    # --------------------------------------------------------

    def calculate_distance(self, task_x, task_y):

        return (
            abs(self.x - task_x)
            + abs(self.y - task_y)
        )

    # --------------------------------------------------------
    # Calculate bid
    # --------------------------------------------------------

    def calculate_bid(
        self,
        priority,
        task_x,
        task_y
    ):

        # Battery <= 15% cannot participate
        if self.battery <= BATTERY_MINIMUM:
            return 0

        # Robot must be available
        if not self.available:
            return 0

        distance = self.calculate_distance(
            task_x,
            task_y
        )

        # Distance score
        distance_score = 100 / (distance + 1)

        # Battery contribution
        battery_score = self.battery / 5

        # Priority contribution
        priority_score = priority * 5

        # Final bid
        bid = (
            distance_score
            + battery_score
            + priority_score
        )

        return round(bid)

    # --------------------------------------------------------
    # Start task
    # --------------------------------------------------------

    def start_task(self, task, work_time):

        self.available = False

        self.current_task = task

        self.busy_until = time.time() + work_time

    # --------------------------------------------------------
    # Check whether task is completed
    # --------------------------------------------------------

    def check_task_completion(self):

        if self.available:
            return False

        if time.time() < self.busy_until:
            return False

        return True

    # --------------------------------------------------------
    # Complete task
    # --------------------------------------------------------

    def complete_task(self):

        if self.current_task is None:
            return

        task_x = self.current_task["x"]
        task_y = self.current_task["y"]

        distance = self.calculate_distance(
            task_x,
            task_y
        )

        # Calculate battery used
        battery_used = (
            distance * BATTERY_PER_DISTANCE
        )

        # Reduce battery
        self.battery = max(
            0,
            self.battery - battery_used
        )

        # Move robot to task location
        self.x = task_x
        self.y = task_y

        completed_task_id = self.current_task["task_id"]

        # Update statistics
        self.completed_tasks += 1

        # Make robot available again
        self.available = True

        self.current_task = None

        self.busy_until = 0

        return (
            completed_task_id,
            distance,
            battery_used
        )


# ============================================================
# CREATE ROBOTS
# ============================================================

def create_robots():

    robots = []

    start_positions = {

        "R1": (12, 4),
        "R2": (22, 4),
        "R3": (32, 4),
        "R4": (42, 4),
        "R5": (52, 4),
        "R6": (62, 4),
        "R7": (72, 4),
        "R8": (82, 4),
        "R9": (88, 4),
        "R10": (48, 12)
    }

    for robot_id, position in start_positions.items():

        x, y = position

        robot = Robot(
            robot_id,
            BATTERY_START,
            x,
            y,
            True
        )

        robots.append(robot)

    return robots


# ============================================================
# CREATE RANDOM TASK
# ============================================================

def create_task(task_number):

    task_x = random.randint(5, 95)

    task_y = random.randint(5, 95)

    priority = random.randint(1, 5)

    return {

        "task_id": f"T{task_number:03}",

        "x": task_x,

        "y": task_y,

        "priority": priority
    }


# ============================================================
# DISPLAY ROBOTS
# ============================================================

def display_robots(
    robots,
    distances,
    bids
):

    print("\nRobot Details")
    print("-" * 105)

    print(
        f"{'Robot':<8}"
        f"{'Battery':<12}"
        f"{'Position':<15}"
        f"{'Distance':<12}"
        f"{'Bid':<8}"
        f"{'Status':<18}"
        f"{'Current Task'}"
    )

    print("-" * 105)

    for robot in robots:

        distance = distances.get(
            robot.robot_id,
            "-"
        )

        bid = bids.get(
            robot.robot_id,
            0
        )

        # Determine status

        if robot.battery <= BATTERY_MINIMUM:

            status = "LOW BATTERY"

        elif not robot.available:

            status = "BUSY"

        else:

            status = "AVAILABLE"

        if robot.current_task:

            current_task = robot.current_task["task_id"]

        else:

            current_task = "-"

        print(
            f"{robot.robot_id:<8}"
            f"{robot.battery:>6.1f}%     "
            f"({robot.x},{robot.y})"
            f"{'':<6}"
            f"{str(distance):<12}"
            f"{bid:<8}"
            f"{status:<18}"
            f"{current_task}"
        )

    print("-" * 105)


# ============================================================
# UPDATE ROBOTS
# ============================================================

def update_robots(robots):

    for robot in robots:

        if not robot.available:

            if robot.check_task_completion():

                result = robot.complete_task()

                if result:

                    (
                        task_id,
                        distance,
                        battery_used
                    ) = result

                    print("\n")
                    print("=" * 90)
                    print("                     TASK COMPLETED")
                    print("=" * 90)

                    print(
                        f"Task           : {task_id}"
                    )

                    print(
                        f"Robot          : {robot.robot_id}"
                    )

                    print(
                        f"Travel Distance: {distance:.2f}"
                    )

                    print(
                        f"Battery Used   : {battery_used:.2f}%"
                    )

                    print(
                        f"Battery Now    : {robot.battery:.2f}%"
                    )

                    print(
                        f"New Position   : "
                        f"({robot.x}, {robot.y})"
                    )

                    print(
                        f"Completed Tasks: "
                        f"{robot.completed_tasks}"
                    )

                    print("=" * 90)


# ============================================================
# AUCTION SYSTEM
# ============================================================

def auction_task(
    robots,
    task
):

    task_x = task["x"]

    task_y = task["y"]

    priority = task["priority"]

    # --------------------------------------------------------
    # Find eligible robots
    # --------------------------------------------------------

    eligible_robots = [

        robot

        for robot in robots

        if robot.battery > BATTERY_MINIMUM

        and robot.available
    ]

    print("\n")
    print("=" * 90)

    print(
        f"                    AUCTION FOR {task['task_id']}"
    )

    print("=" * 90)

    print(
        f"Task Position : ({task_x}, {task_y})"
    )

    print(
        f"Task Priority : {priority}"
    )

    # --------------------------------------------------------
    # No eligible robots
    # --------------------------------------------------------

    if not eligible_robots:

        print(
            "\nNo available robot right now."
        )

        print(
            f"Task {task['task_id']} will WAIT."
        )

        return None

    # --------------------------------------------------------
    # Calculate distances and bids
    # --------------------------------------------------------

    distances = {}

    bids = {}

    for robot in robots:

        distance = robot.calculate_distance(
            task_x,
            task_y
        )

        distances[
            robot.robot_id
        ] = distance

        bids[
            robot.robot_id
        ] = robot.calculate_bid(
            priority,
            task_x,
            task_y
        )

    # --------------------------------------------------------
    # Display robot information
    # --------------------------------------------------------

    display_robots(
        robots,
        distances,
        bids
    )

    # --------------------------------------------------------
    # Find closest eligible robot
    # --------------------------------------------------------

    initial_winner = min(

        eligible_robots,

        key=lambda robot: (

            distances[
                robot.robot_id
            ],

            int(
                robot.robot_id[1:]
            )
        )
    )

    closest_distance = distances[
        initial_winner.robot_id
    ]

    # --------------------------------------------------------
    # +3 distance limit
    # --------------------------------------------------------

    maximum_distance = (
        closest_distance
        + DISTANCE_TOLERANCE
    )

    # --------------------------------------------------------
    # Find better battery robots
    # within +3 distance
    # --------------------------------------------------------

    better_candidates = []

    for robot in eligible_robots:

        if (
            robot.robot_id
            == initial_winner.robot_id
        ):

            continue

        robot_distance = distances[
            robot.robot_id
        ]

        if robot_distance > maximum_distance:

            continue

        if (
            robot.battery
            <= initial_winner.battery
        ):

            continue

        better_candidates.append(
            robot
        )

    # --------------------------------------------------------
    # Select final winner
    # --------------------------------------------------------

    if better_candidates:

        winner = min(

            better_candidates,

            key=lambda robot: (

                -robot.battery,

                distances[
                    robot.robot_id
                ],

                int(
                    robot.robot_id[1:]
                )
            )
        )

        reason = (
            "Higher-battery robot found "
            "within +3 distance."
        )

    else:

        winner = initial_winner

        reason = (
            "No higher-battery robot found "
            "within +3 distance."
        )

    # --------------------------------------------------------
    # Display result
    # --------------------------------------------------------

    print("\n")
    print("=" * 90)
    print("                       AUCTION RESULT")
    print("=" * 90)

    print(
        f"Initial Winner   : "
        f"{initial_winner.robot_id}"
    )

    print(
        f"Initial Battery  : "
        f"{initial_winner.battery:.1f}%"
    )

    print(
        f"Initial Distance : "
        f"{closest_distance:.2f}"
    )

    print(
        f"Allowed Distance : "
        f"{maximum_distance:.2f}"
    )

    print("-" * 90)

    print(
        f"FINAL WINNER     : "
        f"{winner.robot_id}"
    )

    print(
        f"Winner Battery   : "
        f"{winner.battery:.1f}%"
    )

    print(
        f"Winner Distance  : "
        f"{distances[winner.robot_id]:.2f}"
    )

    print(
        f"Winning Bid      : "
        f"{bids[winner.robot_id]}"
    )

    print(
        f"Priority         : "
        f"{priority}"
    )

    print(
        f"Reason           : "
        f"{reason}"
    )

    print("=" * 90)

    # --------------------------------------------------------
    # Start task
    # --------------------------------------------------------

    winner.start_task(
        task,
        WORK_TIME
    )

    print("\n")
    print(
        f"🤖 {winner.robot_id} STARTED "
        f"WORKING ON {task['task_id']}"
    )

    print(
        f"   Work time: {WORK_TIME} seconds"
    )

    print(
        f"   Status: BUSY"
    )

    return winner


# ============================================================
# DISPLAY SUMMARY
# ============================================================

def display_summary(robots):

    print("\n")
    print("=" * 90)
    print("                    ROBOT STATUS")
    print("=" * 90)

    print(
        f"{'Robot':<8}"
        f"{'Battery':<12}"
        f"{'Position':<15}"
        f"{'Status':<15}"
        f"{'Current Task':<15}"
        f"{'Completed'}"
    )

    print("-" * 90)

    for robot in robots:

        if robot.battery <= BATTERY_MINIMUM:

            status = "LOW BATTERY"

        elif robot.available:

            status = "AVAILABLE"

        else:

            status = "BUSY"

        if robot.current_task:

            task_id = robot.current_task["task_id"]

        else:

            task_id = "-"

        print(
            f"{robot.robot_id:<8}"
            f"{robot.battery:>6.1f}%     "
            f"({robot.x},{robot.y})"
            f"{'':<6}"
            f"{status:<15}"
            f"{task_id:<15}"
            f"{robot.completed_tasks}"
        )

    print("=" * 90)


# ============================================================
# MAIN CONTINUOUS SIMULATION
# ============================================================

def main():

    robots = create_robots()

    print("\n")
    print("=" * 90)
    print("                 CONTINUOUS ROBOT BIDDING SYSTEM")
    print("=" * 90)

    print(
        f"Number of Robots   : "
        f"{NUMBER_OF_ROBOTS}"
    )

    print(
        f"Starting Battery   : "
        f"{BATTERY_START}%"
    )

    print(
        f"Minimum Battery    : "
        f"{BATTERY_MINIMUM}%"
    )

    print(
        f"Work Time          : "
        f"{WORK_TIME} seconds"
    )

    print(
        f"New Task Every     : "
        f"{TASK_ARRIVAL_INTERVAL} seconds"
    )

    print(
        "\nPress Ctrl+C to stop the simulation."
    )

    print("=" * 90)

    # --------------------------------------------------------
    # Task counter
    # --------------------------------------------------------

    task_number = 1

    # --------------------------------------------------------
    # Tasks waiting for a robot
    # --------------------------------------------------------

    pending_tasks = []

    # --------------------------------------------------------
    # Create 10 initial tasks
    # --------------------------------------------------------

    print("\n")
    print("=" * 90)
    print("                   INITIAL TASKS")
    print("=" * 90)

    for _ in range(INITIAL_TASKS):

        task = create_task(
            task_number
        )

        task_number += 1

        pending_tasks.append(
            task
        )

        print(
            f"{task['task_id']} "
            f"-> Position: "
            f"({task['x']}, {task['y']}) "
            f"Priority: {task['priority']}"
        )

    print("=" * 90)

    # --------------------------------------------------------
    # Main continuous loop
    # --------------------------------------------------------

    last_task_time = time.time()

    try:

        while True:

            # ================================================
            # Check whether busy robots finished
            # ================================================

            update_robots(
                robots
            )

            # ================================================
            # Try waiting tasks
            # ================================================

            if pending_tasks:

                remaining_tasks = []

                for task in pending_tasks:

                    result = auction_task(
                        robots,
                        task
                    )

                    if result is None:

                        # Nobody available.
                        # Keep the task waiting.

                        remaining_tasks.append(
                            task
                        )

                    # If robot accepted task,
                    # do not put it back.

                pending_tasks = remaining_tasks

            # ================================================
            # Create new task automatically
            # ================================================

            current_time = time.time()

            if (
                current_time - last_task_time
                >= TASK_ARRIVAL_INTERVAL
            ):

                task = create_task(
                    task_number
                )

                task_number += 1

                last_task_time = current_time

                print("\n")
                print("=" * 90)
                print("                    NEW TASK ARRIVED")
                print("=" * 90)

                print(
                    f"Task ID  : "
                    f"{task['task_id']}"
                )

                print(
                    f"Position : "
                    f"({task['x']}, {task['y']})"
                )

                print(
                    f"Priority : "
                    f"{task['priority']}"
                )

                print("=" * 90)

                # Add to waiting list
                pending_tasks.append(
                    task
                )

            # ================================================
            # Small delay
            # ================================================

            time.sleep(0.5)

    except KeyboardInterrupt:

        print("\n\n")
        print("=" * 90)
        print("                SIMULATION STOPPED")
        print("=" * 90)

        display_summary(
            robots
        )

        print("\nThank you for using the Robot Bidding System.")


# ============================================================
# START PROGRAM
# ============================================================

if __name__ == "__main__":
    main()
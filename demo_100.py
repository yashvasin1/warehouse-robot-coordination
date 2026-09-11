import contextlib
import io
import random
import pygame

from bidding import Robot as AuctionRobot
from bidding import auction_task, BATTERY_MINIMUM
from coordinator_final import MultiRobotCoordinator


# ============================================================
# CONFIGURATION
# ============================================================

GRID_COLUMNS = 100
GRID_ROWS = 100
CELL_SIZE = 8

WIDTH = GRID_COLUMNS * CELL_SIZE
HEIGHT = GRID_ROWS * CELL_SIZE

COMMUNICATION_RANGE = 20

NUMBER_OF_ROBOTS = 10

# Total jobs for the demonstration.
TOTAL_TASKS = 30

RANDOM_SEED = 42

# Slower movement makes every grid-cell movement visible.
STEP_DELAY_MS = 220

# Robot waits visually for two coordinator steps at the pickup point.
PICKUP_DWELL_STEPS = 2

# Demo-side liveness guard.
# If a robot remains waiting for too long, force a fresh
# reservation-aware route calculation.
STALL_LIMIT = 7


# ============================================================
# COLORS
# ============================================================

BACKGROUND = (28, 33, 38)
GRID_COLOR = (48, 55, 60)
WALL_COLOR = (120, 128, 138)
RACK_COLOR = (155, 95, 42)
RACK_EDGE = (205, 140, 70)

PICKUP_COLOR = (40, 175, 90)
DROP_COLOR = (50, 115, 215)
CHARGING_COLOR = (215, 175, 40)
INBOUND_COLOR = (45, 150, 180)
OUTBOUND_COLOR = (170, 75, 180)

ROBOT_COLOR = (215, 65, 65)
CARGO_COLOR = (255, 220, 80)
PATH_COLOR = (105, 135, 160)
TEXT_COLOR = (235, 235, 235)
HUD_BG = (10, 12, 15)
WAIT_COLOR = (255, 170, 60)


# ============================================================
# 100x100 WAREHOUSE
# ============================================================

def build_warehouse():
    walls = set()
    racks = set()
    pickup_zones = set()
    drop_zones = set()
    charging_stations = set()
    inbound_zones = set()
    outbound_zones = set()

    # Boundary walls.
    for col in range(GRID_COLUMNS):
        walls.add((col, 0))
        walls.add((col, GRID_ROWS - 1))

    for row in range(GRID_ROWS):
        walls.add((0, row))
        walls.add((GRID_COLUMNS - 1, row))

    # Same rack structure as the supplied 100x100 template.
    rack_rows = [
        8, 9,
        16, 17,
        24, 25,
        32, 33,
        40, 41,
        48, 49,
        56, 57,
        64, 65,
        72, 73,
        80, 81,
    ]

    aisle_gaps = {28, 29, 50, 51, 72, 73}

    for row in rack_rows:
        for col in range(8, 92):
            if col not in aisle_gaps:
                racks.add((col, row))

    # Operational zones.
    for col in range(3, 8):
        for row in range(3, 8):
            inbound_zones.add((col, row))

    for col in range(93, 98):
        for row in range(3, 8):
            outbound_zones.add((col, row))

    for col in range(3, 8):
        for row in range(92, 97):
            pickup_zones.add((col, row))

    for col in range(93, 98):
        for row in range(92, 97):
            drop_zones.add((col, row))

    for col in range(46, 55):
        for row in range(92, 97):
            charging_stations.add((col, row))

    robot_positions = [
        (12, 4),
        (22, 4),
        (32, 4),
        (42, 4),
        (52, 4),
        (62, 4),
        (72, 4),
        (82, 4),
        (88, 4),
        (48, 12),
    ]

    return {
        "walls": walls,
        "racks": racks,
        "pickup_zones": pickup_zones,
        "drop_zones": drop_zones,
        "charging_stations": charging_stations,
        "inbound_zones": inbound_zones,
        "outbound_zones": outbound_zones,
        "robot_positions": robot_positions,
    }


def build_planner_map(warehouse):
    blocked = warehouse["walls"] | warehouse["racks"]

    return [
        "".join(
            "#" if (x, y) in blocked else "."
            for x in range(GRID_COLUMNS)
        )
        for y in range(GRID_ROWS)
    ]


# ============================================================
# AUCTION ROBOT ADAPTER
# ============================================================

class NavigationAuctionRobot(AuctionRobot):
    """
    Uses the friend's auction calculation, but physical movement
    is completely controlled by the coordinator.
    """

    def start_task(self, task, work_time):
        self.available = False
        self.current_task = task
        self.busy_until = 0


# ============================================================
# TASK GENERATION
# ============================================================

def get_rack_pickup_cells(warehouse):
    blocked = warehouse["walls"] | warehouse["racks"]
    candidates = set()

    for rack_x, rack_y in warehouse["racks"]:
        for dx, dy in (
            (1, 0),
            (-1, 0),
            (0, 1),
            (0, -1),
        ):
            cell = (rack_x + dx, rack_y + dy)

            if (
                0 <= cell[0] < GRID_COLUMNS
                and 0 <= cell[1] < GRID_ROWS
                and cell not in blocked
            ):
                candidates.add(cell)

    return candidates


def _manhattan(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def _choose_spread_cell(candidates, active_targets, rng, min_distance):
    """
    Choose a cell that is not too close to currently active task targets.

    This prevents all ten robots from being sent to the same rack or the
    same tiny section of the drop area, which was creating artificial
    traffic jams in the visual demo.
    """
    candidates = list(candidates)

    if not candidates:
        return None

    rng.shuffle(candidates)

    if not active_targets:
        return rng.choice(candidates)

    spread = [
        cell
        for cell in candidates
        if all(
            _manhattan(cell, target) >= min_distance
            for target in active_targets
        )
    ]

    if spread:
        # Prefer a cell farthest from the existing active targets.
        spread.sort(
            key=lambda cell: (
                min(_manhattan(cell, target) for target in active_targets),
                cell[1],
                cell[0],
            ),
            reverse=True,
        )
        return spread[0]

    # Graceful fallback when the warehouse is crowded.
    candidates.sort(
        key=lambda cell: (
            min(_manhattan(cell, target) for target in active_targets),
            cell[1],
            cell[0],
        ),
        reverse=True,
    )

    return candidates[0]


def create_task(
    warehouse,
    coordinator,
    task_number,
    rng,
    active_jobs=None,
):
    """
    Create a physically realistic warehouse job:

        unique rack-adjacent PICKUP
                    ↓
                 robot
                    ↓
        unique DROP / OUTBOUND slot

    Active tasks reserve their pickup and drop targets so multiple robots do
    not intentionally converge on the same destination.
    """
    if active_jobs is None:
        active_jobs = {}

    pickup_candidates = get_rack_pickup_cells(warehouse)

    robot_positions = {
        robot.position
        for robot in coordinator.robots.values()
    }

    active_pickups = {
        job["task"]["pickup"]
        for job in active_jobs.values()
    }

    active_drops = {
        job["task"]["drop"]
        for job in active_jobs.values()
    }

    # Never reuse an active pickup or drop target.
    pickup_candidates -= robot_positions
    pickup_candidates -= active_pickups

    drop_candidates = (
        set(warehouse["drop_zones"])
        | set(warehouse["outbound_zones"])
    )

    drop_candidates -= robot_positions
    drop_candidates -= active_drops

    if not pickup_candidates:
        raise RuntimeError(
            "No unique rack pickup location is available."
        )

    if not drop_candidates:
        raise RuntimeError(
            "No unique drop/outbound location is available."
        )

    # Spread active pickup targets across different rack sections.
    pickup = _choose_spread_cell(
        pickup_candidates,
        list(active_pickups),
        rng,
        min_distance=8,
    )

    # Spread drop targets across the large bottom operational area.
    drop = _choose_spread_cell(
        drop_candidates,
        list(active_drops),
        rng,
        min_distance=10,
    )

    return {
        "task_id": f"T{task_number:03}",
        "pickup": pickup,
        "drop": drop,
        "x": pickup[0],
        "y": pickup[1],
        "priority": rng.randint(1, 5),
    }



def reset_robot_for_new_job(coordinator, auction_robots, robot_id):
    """
    Fully clear the previous job state before assigning a new job.

    This is the explicit reset that prevents stale A* paths, reservations,
    wait/yield state, and destination state from leaking into the next task.
    """
    robot = coordinator.robots[robot_id]

    # Clear this robot's reservation entries first.
    coordinator.reservations.clear_robot(robot_id)

    # Clear every navigation field from the previous job.
    robot.path = []
    robot.destination = None
    robot.status = "IDLE"
    robot.wait_steps = 0
    robot.waiting_for = None

    # Clear local congestion state when present.
    if hasattr(robot, "blocked_steps"):
        robot.blocked_steps = 0
    if hasattr(robot, "blocked_target"):
        robot.blocked_target = None
    if hasattr(robot, "blocked_target_until"):
        robot.blocked_target_until = 0
    if hasattr(robot, "last_blocker"):
        robot.last_blocker = None

    # Reset auction-side state.
    auction_robot = auction_robots[robot_id]
    auction_robot.available = True
    auction_robot.current_task = None
    auction_robot.busy_until = 0
    auction_robot.x = robot.position[0]
    auction_robot.y = robot.position[1]
    auction_robot.battery = robot.battery


def clear_completed_job(coordinator, auction_robots, jobs, robot_id):
    """
    Final cleanup after DROP. No old path, reservation, or task state is
    allowed to survive into the next auction.
    """
    coordinator.reservations.clear_robot(robot_id)

    robot = coordinator.robots[robot_id]
    robot.path = []
    robot.destination = None
    robot.status = "IDLE"
    robot.wait_steps = 0
    robot.waiting_for = None

    for attr, value in (
        ("blocked_steps", 0),
        ("blocked_target", None),
        ("blocked_target_until", 0),
        ("last_blocker", None),
    ):
        if hasattr(robot, attr):
            setattr(robot, attr, value)

    auction_robot = auction_robots[robot_id]
    auction_robot.available = True
    auction_robot.current_task = None
    auction_robot.busy_until = 0
    auction_robot.x = robot.position[0]
    auction_robot.y = robot.position[1]
    auction_robot.battery = robot.battery

    jobs.pop(robot_id, None)



# ============================================================
# AUCTION
# ============================================================

def build_auction_robots(coordinator):
    result = {}

    for robot_id, robot in coordinator.robots.items():
        result[robot_id] = NavigationAuctionRobot(
            robot_id=robot_id,
            battery=robot.battery,
            x=robot.position[0],
            y=robot.position[1],
            available=True,
        )

    return result


def sync_auction_robots(coordinator, auction_robots):
    for robot_id, auction_robot in auction_robots.items():
        robot = coordinator.robots[robot_id]

        auction_robot.x = robot.position[0]
        auction_robot.y = robot.position[1]
        auction_robot.battery = robot.battery

        auction_robot.available = (
            robot.status == "IDLE"
            and robot.battery > BATTERY_MINIMUM
        )

        auction_robot.current_task = None
        auction_robot.busy_until = 0


def auction_one_task(
    coordinator,
    auction_robots,
    task,
):
    eligible = [
        robot
        for robot in auction_robots.values()
        if robot.available
        and robot.battery > BATTERY_MINIMUM
    ]

    if not eligible:
        return None

    bid_snapshot = {
        robot.robot_id: robot.calculate_bid(
            task["priority"],
            task["x"],
            task["y"],
        )
        for robot in eligible
    }

    # Reuse the friend's actual auction logic.
    with contextlib.redirect_stdout(io.StringIO()):
        winner = auction_task(
            list(auction_robots.values()),
            task,
        )

    if winner is None:
        return None

    winner_id = winner.robot_id

    # IMPORTANT: clear every trace of the robot's previous task before
    # starting the new pickup trip.
    reset_robot_for_new_job(
        coordinator,
        auction_robots,
        winner_id,
    )

    coordinator.assign_destination(
        winner_id,
        task["pickup"],
        priority=task["priority"],
    )

    auction_robots[winner_id].available = False

    print(
        f"[AUCTION] {task['task_id']} -> {winner_id} | "
        f"priority={task['priority']} | "
        f"bid={bid_snapshot.get(winner_id, 0)} | "
        f"PICKUP={task['pickup']} | "
        f"DROP={task['drop']}"
    )

    return {
        "task": task,
        "robot_id": winner_id,
        "phase": "TO_PICKUP",
        "pickup_dwell": 0,
        "carrying": False,
        "stall_count": 0,
    }


# ============================================================
# PICKUP / DROP STATE MACHINE
# ============================================================


def get_available_drop_cells(warehouse, coordinator, exclude_robot_id=None):
    """
    Return drop/outbound cells that are currently free and not already
    committed as a drop target by another active job.

    A drop point is a physical destination cell, so it must remain free when
    the carrying robot reaches it.
    """
    occupied = {
        robot.position
        for rid, robot in coordinator.robots.items()
        if rid != exclude_robot_id
    }

    return (
        set(warehouse["drop_zones"])
        | set(warehouse["outbound_zones"])
    ) - occupied


def choose_reachable_drop(
    warehouse,
    coordinator,
    robot_id,
    preferred_drop,
    jobs,
):
    """
    Choose a free, reachable drop location.

    Preference:
      1. Keep the originally assigned drop if it is still free and reachable.
      2. Otherwise choose the nearest currently free reachable drop cell.

    This prevents a finished robot from permanently blocking the last job.
    """
    robot = coordinator.robots[robot_id]

    active_drop_cells = {
        job["task"]["drop"]
        for rid, job in jobs.items()
        if rid != robot_id
        and job["phase"] == "TO_DROP"
    }

    candidates = get_available_drop_cells(
        warehouse,
        coordinator,
        exclude_robot_id=robot_id,
    )

    candidates -= active_drop_cells

    if preferred_drop in candidates:
        path = coordinator.planner.find_path(
            robot.position,
            preferred_drop,
            blocked_cells=coordinator.get_blocked_positions(robot_id),
        )

        if path is not None:
            return preferred_drop

    best = None
    best_key = None

    for candidate in candidates:
        path = coordinator.planner.find_path(
            robot.position,
            candidate,
            blocked_cells=coordinator.get_blocked_positions(robot_id),
        )

        if path is None:
            continue

        # Prefer shorter travel; tie-break deterministically by coordinates.
        key = (len(path), candidate[1], candidate[0])

        if best_key is None or key < best_key:
            best_key = key
            best = candidate

    return best


def refresh_drop_target(
    warehouse,
    coordinator,
    auction_robots,
    jobs,
    robot_id,
):
    """
    If the robot's assigned drop becomes blocked, immediately select another
    reachable free drop cell and restart the second A* leg.
    """
    job = jobs[robot_id]
    task = job["task"]
    robot = coordinator.robots[robot_id]

    new_drop = choose_reachable_drop(
        warehouse,
        coordinator,
        robot_id,
        task["drop"],
        jobs,
    )

    if new_drop is None:
        return False

    if new_drop != task["drop"]:
        print(
            f"[DROP REROUTE] {robot_id}: original drop {task['drop']} "
            f"is unavailable -> new drop {new_drop}"
        )

        task["drop"] = new_drop

    robot.status = "IDLE"
    robot.destination = None
    robot.path = []
    robot.wait_steps = 0
    robot.waiting_for = None

    coordinator.assign_destination(
        robot_id,
        task["drop"],
        priority=task["priority"],
    )

    auction_robots[robot_id].available = False
    auction_robots[robot_id].x = robot.position[0]
    auction_robots[robot_id].y = robot.position[1]
    auction_robots[robot_id].battery = robot.battery

    job["phase"] = "TO_DROP"
    job["carrying"] = True
    job["stall_count"] = 0

    return True


def process_jobs(
    warehouse,
    coordinator,
    auction_robots,
    jobs,
):
    completed = []

    for robot_id, job in list(jobs.items()):
        robot = coordinator.robots[robot_id]
        task = job["task"]

        if robot.status == "WAITING":
            job["stall_count"] += 1
        else:
            job["stall_count"] = 0

        # --------------------------------------------------------
        # TO PICKUP
        # --------------------------------------------------------
        if (
            job["phase"] == "TO_PICKUP"
            and robot.status == "ARRIVED"
        ):
            job["phase"] = "PICKING_UP"
            job["pickup_dwell"] = PICKUP_DWELL_STEPS

            print(
                f"[PICKUP ARRIVED] {robot_id} reached "
                f"{task['pickup']} for {task['task_id']}"
            )

        # --------------------------------------------------------
        # PICKUP
        # --------------------------------------------------------
        if job["phase"] == "PICKING_UP":
            if job["pickup_dwell"] > 0:
                job["pickup_dwell"] -= 1

            else:
                new_drop = choose_reachable_drop(
                    warehouse,
                    coordinator,
                    robot_id,
                    task["drop"],
                    jobs,
                )

                if new_drop is None:
                    print(
                        f"[DROP WAIT] {robot_id}: no free reachable "
                        "drop cell right now."
                    )
                    continue

                if new_drop != task["drop"]:
                    print(
                        f"[DROP TARGET UPDATE] {robot_id}: "
                        f"{task['drop']} -> {new_drop}"
                    )
                    task["drop"] = new_drop

                job["phase"] = "TO_DROP"
                job["carrying"] = True
                job["stall_count"] = 0

                coordinator.reservations.clear_robot(robot_id)

                robot.status = "IDLE"
                robot.destination = None
                robot.path = []
                robot.wait_steps = 0
                robot.waiting_for = None

                coordinator.assign_destination(
                    robot_id,
                    task["drop"],
                    priority=task["priority"],
                )

                print(
                    f"[PICKUP COMPLETE] {robot_id} picked up "
                    f"{task['task_id']} -> DROP {task['drop']}"
                )

        # --------------------------------------------------------
        # TO DROP
        # --------------------------------------------------------
        if job["phase"] == "TO_DROP":

            # A drop cell may become occupied after the task was created.
            # Detect that before treating the robot's WAITING state as a
            # permanent condition.
            occupants = coordinator.get_current_occupants(
                task["drop"],
                exclude_id=robot_id,
            )

            if occupants:
                new_drop = choose_reachable_drop(
                    warehouse,
                    coordinator,
                    robot_id,
                    task["drop"],
                    jobs,
                )

                if new_drop is not None and new_drop != task["drop"]:
                    print(
                        f"[DROP REROUTE] {robot_id}: drop {task['drop']} "
                        f"occupied by {occupants} -> {new_drop}"
                    )

                    task["drop"] = new_drop

                    robot.status = "IDLE"
                    robot.destination = None
                    robot.path = []

                    coordinator.assign_destination(
                        robot_id,
                        task["drop"],
                        priority=task["priority"],
                    )

                    job["stall_count"] = 0
                    continue

            # If the robot has been waiting too long, choose a fresh drop
            # rather than endlessly retrying a blocked destination.
            if (
                robot.status == "WAITING"
                and job["stall_count"] >= STALL_LIMIT
            ):
                if refresh_drop_target(
                    warehouse,
                    coordinator,
                    auction_robots,
                    jobs,
                    robot_id,
                ):
                    continue

            if robot.status == "ARRIVED":
                print(
                    f"[DROP COMPLETE] {robot_id} delivered "
                    f"{task['task_id']} at {task['drop']}"
                )

                job["phase"] = "COMPLETED"
                job["carrying"] = False

                # Fully clear the finished task before the robot re-enters
                # the auction pool.
                clear_completed_job(
                    coordinator,
                    auction_robots,
                    jobs,
                    robot_id,
                )

                completed.append(robot_id)

    for robot_id in completed:
        jobs.pop(robot_id, None)

    return len(completed)




# ============================================================
# DRAWING
# ============================================================

def cell_rect(cell):
    x, y = cell

    return pygame.Rect(
        x * CELL_SIZE,
        y * CELL_SIZE,
        CELL_SIZE,
        CELL_SIZE,
    )


def draw_cells(screen, cells, color):
    for cell in cells:
        pygame.draw.rect(
            screen,
            color,
            cell_rect(cell).inflate(-2, -2),
        )


def draw_racks(screen, racks):
    for cell in racks:
        rect = cell_rect(cell)

        pygame.draw.rect(
            screen,
            RACK_COLOR,
            rect.inflate(-2, -2),
        )

        pygame.draw.rect(
            screen,
            RACK_EDGE,
            rect.inflate(-2, -2),
            1,
        )


def draw_walls(screen, walls):
    for cell in walls:
        pygame.draw.rect(
            screen,
            WALL_COLOR,
            cell_rect(cell).inflate(-1, -1),
        )


def draw_task_markers(screen, jobs):
    font = pygame.font.SysFont("arial", 9, bold=True)

    for job in jobs.values():
        task = job["task"]

        pickup = task["pickup"]
        drop = task["drop"]

        pickup_center = (
            pickup[0] * CELL_SIZE + CELL_SIZE // 2,
            pickup[1] * CELL_SIZE + CELL_SIZE // 2,
        )

        drop_center = (
            drop[0] * CELL_SIZE + CELL_SIZE // 2,
            drop[1] * CELL_SIZE + CELL_SIZE // 2,
        )

        if job["phase"] in ("TO_PICKUP", "PICKING_UP"):
            pygame.draw.circle(
                screen,
                PICKUP_COLOR,
                pickup_center,
                max(5, CELL_SIZE // 2),
                2,
            )

            label = font.render(
                f"P:{task['task_id']}",
                True,
                TEXT_COLOR,
            )

            screen.blit(
                label,
                (
                    pickup_center[0] + 5,
                    pickup_center[1] - 5,
                ),
            )

        if job["phase"] == "TO_DROP":
            pygame.draw.rect(
                screen,
                DROP_COLOR,
                cell_rect(drop).inflate(-1, -1),
                2,
            )

            label = font.render(
                f"D:{task['task_id']}",
                True,
                TEXT_COLOR,
            )

            screen.blit(
                label,
                (
                    drop_center[0] + 5,
                    drop_center[1] - 5,
                ),
            )


def draw_paths(screen, coordinator, jobs):
    for robot_id in jobs:
        robot = coordinator.robots[robot_id]

        if robot.status not in ("MOVING", "WAITING"):
            continue

        if not robot.path:
            continue

        points = [
            (
                robot.position[0] * CELL_SIZE + CELL_SIZE // 2,
                robot.position[1] * CELL_SIZE + CELL_SIZE // 2,
            )
        ]

        for cell in robot.path:
            points.append(
                (
                    cell[0] * CELL_SIZE + CELL_SIZE // 2,
                    cell[1] * CELL_SIZE + CELL_SIZE // 2,
                )
            )

        if len(points) >= 2:
            pygame.draw.lines(
                screen,
                PATH_COLOR,
                False,
                points,
                2,
            )


def draw_mesh(screen, coordinator):
    drawn = set()

    for robot_id in coordinator.mesh.positions:
        for other_id in coordinator.mesh.get_neighbors(robot_id):

            if other_id not in coordinator.mesh.positions:
                continue

            pair = tuple(sorted((robot_id, other_id)))

            if pair in drawn:
                continue

            drawn.add(pair)

            p1 = coordinator.robots[robot_id].position
            p2 = coordinator.robots[other_id].position

            pygame.draw.line(
                screen,
                (75, 95, 110),
                (
                    p1[0] * CELL_SIZE + CELL_SIZE // 2,
                    p1[1] * CELL_SIZE + CELL_SIZE // 2,
                ),
                (
                    p2[0] * CELL_SIZE + CELL_SIZE // 2,
                    p2[1] * CELL_SIZE + CELL_SIZE // 2,
                ),
                1,
            )


def draw_robots(screen, coordinator, jobs):
    font = pygame.font.SysFont(
        "arial",
        11,
        bold=True,
    )

    for robot_id, robot in coordinator.robots.items():
        center = (
            robot.position[0] * CELL_SIZE + CELL_SIZE // 2,
            robot.position[1] * CELL_SIZE + CELL_SIZE // 2,
        )

        pygame.draw.circle(
            screen,
            ROBOT_COLOR,
            center,
            max(5, CELL_SIZE // 2),
        )

        job = jobs.get(robot_id)

        if job and job["carrying"]:
            pygame.draw.circle(
                screen,
                CARGO_COLOR,
                center,
                max(7, CELL_SIZE // 2 + 2),
                2,
            )

        if robot.status == "WAITING":
            pygame.draw.circle(
                screen,
                WAIT_COLOR,
                center,
                max(8, CELL_SIZE // 2 + 4),
                2,
            )

        text = font.render(
            robot_id,
            True,
            TEXT_COLOR,
        )

        screen.blit(
            text,
            text.get_rect(center=center),
        )


def draw_hud(
    screen,
    coordinator,
    jobs,
    completed_total,
):
    pygame.draw.rect(
        screen,
        HUD_BG,
        pygame.Rect(0, 0, WIDTH, 58),
    )

    title_font = pygame.font.SysFont(
        "arial",
        15,
        bold=True,
    )

    small_font = pygame.font.SysFont(
        "arial",
        11,
    )

    moving = sum(
        1
        for robot in coordinator.robots.values()
        if robot.status == "MOVING"
    )

    waiting = sum(
        1
        for robot in coordinator.robots.values()
        if robot.status == "WAITING"
    )

    pickup = sum(
        1
        for job in jobs.values()
        if job["phase"] in ("TO_PICKUP", "PICKING_UP")
    )

    drop = sum(
        1
        for job in jobs.values()
        if job["phase"] == "TO_DROP"
    )

    carrying = sum(
        1
        for job in jobs.values()
        if job["carrying"]
    )

    waiting = sum(
        1
        for robot in coordinator.robots.values()
        if robot.status == "WAITING"
    )

    title = title_font.render(
        "100x100 WAREHOUSE | PICKUP -> DROP | "
        f"RANGE {COMMUNICATION_RANGE}",
        True,
        TEXT_COLOR,
    )

    stats = small_font.render(
        f"Assigned: {len(jobs)} | "
        f"Moving: {moving} | "
        f"Waiting: {waiting} | "
        f"To Pickup: {pickup} | "
        f"Carrying: {carrying} | "
        f"To Drop: {drop} | "
        f"Completed: {completed_total}/{TOTAL_TASKS}",
        True,
        TEXT_COLOR,
    )

    screen.blit(title, (8, 5))
    screen.blit(stats, (8, 30))


# ============================================================
# MAIN
# ============================================================

def main():
    pygame.init()

    screen = pygame.display.set_mode(
        (WIDTH, HEIGHT)
    )

    pygame.display.set_caption(
        "100x100 Warehouse - Continuous Pickup and Drop"
    )

    clock = pygame.time.Clock()

    warehouse = build_warehouse()
    planner_map = build_planner_map(warehouse)

    coordinator = MultiRobotCoordinator(
        planner_map,
        communication_range=COMMUNICATION_RANGE,
    )

    for number, position in enumerate(
        warehouse["robot_positions"],
        start=1,
    ):
        coordinator.add_robot(
            f"R{number}",
            position,
            priority=1,
            battery=100.0,
        )

    auction_robots = build_auction_robots(
        coordinator
    )

    rng = random.Random(RANDOM_SEED)

    jobs = {}
    completed_total = 0
    next_task_number = 1

    running = True
    paused = False
    finished = False

    last_step_time = 0

    print("=" * 80)
    print("100x100 DECENTRALIZED CONTINUOUS PICKUP/DROP DEMO")
    print("=" * 80)
    print("Warehouse            : 100 x 100")
    print("Robots               : 10")
    print("Communication range  : 20")
    print("Multi-hop mesh        : ENABLED")
    print("Task type             : RACK PICKUP -> DROP")
    print(f"Total demo tasks      : {TOTAL_TASKS}")
    print()
    print("A task is complete ONLY after reaching its DROP location.")
    print("ESC = quit | SPACE = pause")
    print("=" * 80)

    while running:

        now = pygame.time.get_ticks()

        for event in pygame.event.get():

            if event.type == pygame.QUIT:
                running = False

            elif event.type == pygame.KEYDOWN:

                if event.key == pygame.K_ESCAPE:
                    running = False

                elif event.key == pygame.K_SPACE:
                    paused = not paused

        if not paused and not finished:

            # ------------------------------------------------
            # KEEP ALL AVAILABLE ROBOTS PRODUCTIVELY ASSIGNED
            # ------------------------------------------------
            sync_auction_robots(
                coordinator,
                auction_robots,
            )

            while (
                len(jobs) < NUMBER_OF_ROBOTS
                and next_task_number <= TOTAL_TASKS
            ):
                task = create_task(
                    warehouse,
                    coordinator,
                    next_task_number,
                    rng,
                    active_jobs=jobs,
                )

                next_task_number += 1

                job = auction_one_task(
                    coordinator,
                    auction_robots,
                    task,
                )

                if job is None:
                    break

                # The winning robot is now fully reset and assigned exactly
                # one new pickup/drop job.
                jobs[job["robot_id"]] = job

            # ------------------------------------------------
            # ADVANCE THE REAL COORDINATOR
            # ------------------------------------------------
            if (
                jobs
                and now - last_step_time >= STEP_DELAY_MS
            ):
                last_step_time = now

                for robot_id in coordinator.robots:

                    if (
                        coordinator.robots[robot_id].status
                        != "FAILED"
                    ):
                        coordinator.heartbeat(robot_id)

                coordinator.step()

                completed_now = process_jobs(
                    warehouse,
                    coordinator,
                    auction_robots,
                    jobs,
                )

                completed_total += completed_now

                sync_auction_robots(
                    coordinator,
                    auction_robots,
                )

            # ------------------------------------------------
            # FINISH
            # ------------------------------------------------
            if (
                completed_total >= TOTAL_TASKS
                and not jobs
            ):
                finished = True

                print()
                print("=" * 80)
                print("FINAL DEMO RESULT")
                print("=" * 80)
                print(
                    f"Tasks completed : {completed_total}/{TOTAL_TASKS}"
                )
                print(
                    f"Coordinator steps: {coordinator.time_step}"
                )
                print("RESULT           : PASS")
                print("=" * 80)

        # ----------------------------------------------------
        # DRAW
        # ----------------------------------------------------
        screen.fill(BACKGROUND)

        draw_cells(
            screen,
            warehouse["inbound_zones"],
            INBOUND_COLOR,
        )

        draw_cells(
            screen,
            warehouse["outbound_zones"],
            OUTBOUND_COLOR,
        )

        draw_cells(
            screen,
            warehouse["pickup_zones"],
            PICKUP_COLOR,
        )

        draw_cells(
            screen,
            warehouse["drop_zones"],
            DROP_COLOR,
        )

        draw_cells(
            screen,
            warehouse["charging_stations"],
            CHARGING_COLOR,
        )

        draw_racks(
            screen,
            warehouse["racks"],
        )

        draw_walls(
            screen,
            warehouse["walls"],
        )

        draw_mesh(
            screen,
            coordinator,
        )

        draw_task_markers(
            screen,
            jobs,
        )

        draw_paths(
            screen,
            coordinator,
            jobs,
        )

        draw_robots(
            screen,
            coordinator,
            jobs,
        )

        draw_hud(
            screen,
            coordinator,
            jobs,
            completed_total,
        )

        pygame.display.flip()
        clock.tick(60)

    pygame.quit()


if __name__ == "__main__":
    main()
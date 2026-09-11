# coordinator.py - V4.4.0 (Priority Shoving & Deadlock Proofing)

from dataclasses import dataclass, field
from path_planner import PathPlanner
from mesh_network_v43_final import MeshNetwork
from reservation import ReservationTable


# ============================================================
# ROBOT
# ============================================================

@dataclass
class Robot:
    robot_id: str
    position: tuple
    destination: tuple | None = None
    path: list = field(default_factory=list)
    status: str = "IDLE"
    battery: float = 100.0
    priority: int = 1
    wait_steps: int = 0
    waiting_for: str | None = None
    reroute_count: int = 0
    last_position: tuple | None = None
    last_heartbeat_step: int = 0
    failure_reason: str | None = None


# ============================================================
# COORDINATOR
# ============================================================

class MultiRobotCoordinator:

    def __init__(self, warehouse_map, communication_range=20):
        self.planner = PathPlanner(warehouse_map)
        self.mesh = MeshNetwork(communication_range)
        self.robots = {}
        self.time_step = 0
        self.reservations = ReservationTable()

        # Deadlock / progress control
        self.no_progress_steps = 0
        self.deadlock_threshold = 5
        self.max_wait_steps = 8
        self.max_reroutes_before_recovery = 3
        self._last_positions = {}

        self.failure_monitoring_enabled = False
        self.heartbeat_timeout = 3
        self.failed_robot_positions = {}

    # ========================================================
    # ROBOT MANAGEMENT
    # ========================================================

    def add_robot(self, robot_id, position, priority=1, battery=100.0):
        robot = Robot(
            robot_id=robot_id,
            position=position,
            priority=priority,
            battery=battery,
            last_position=position,
            last_heartbeat_step=self.time_step,
        )

        self.robots[robot_id] = robot
        self.mesh.add_robot(robot_id, position)
        self._last_positions[robot_id] = position

    def assign_destination(self, robot_id, destination, priority=None):
        robot = self.robots[robot_id]
        robot.destination = destination

        if priority is not None:
            robot.priority = priority

        robot.status = "PLANNING"
        robot.wait_steps = 0
        robot.waiting_for = None
        robot.reroute_count = 0

        self.plan_robot(robot_id)

    # ========================================================
    # V4 FAILURE DETECTION / HEARTBEATS
    # ========================================================

    def enable_failure_monitoring(self, timeout_steps=3):
        if timeout_steps < 1:
            raise ValueError("timeout_steps must be at least 1")

        self.heartbeat_timeout = timeout_steps
        self.failure_monitoring_enabled = True

        for robot in self.robots.values():
            if robot.status != "FAILED":
                robot.last_heartbeat_step = self.time_step

        print(
            f"[HEARTBEAT] Failure monitoring enabled "
            f"(timeout={timeout_steps} steps)"
        )

    def disable_failure_monitoring(self):
        self.failure_monitoring_enabled = False
        print("[HEARTBEAT] Failure monitoring disabled")

    def heartbeat(self, robot_id):
        if robot_id not in self.robots:
            return False

        robot = self.robots[robot_id]
        if robot.status == "FAILED":
            return False

        robot.last_heartbeat_step = self.time_step
        return True

    def check_heartbeats(self):
        if not self.failure_monitoring_enabled:
            return []

        failed_ids = []

        for robot_id, robot in list(self.robots.items()):
            if robot.status == "FAILED":
                continue

            elapsed = self.time_step - robot.last_heartbeat_step
            if elapsed > self.heartbeat_timeout:
                print(
                    f"[HEARTBEAT TIMEOUT] {robot_id} silent for "
                    f"{elapsed} steps"
                )
                if self.fail_robot(
                    robot_id,
                    reason=f"heartbeat timeout ({elapsed} steps)",
                ):
                    failed_ids.append(robot_id)

        return failed_ids

    def fail_robot(self, robot_id, reason="manual failure"):
        if robot_id not in self.robots:
            return False

        robot = self.robots[robot_id]
        if robot.status == "FAILED":
            return False

        old_position = robot.position
        self.reservations.clear_robot(robot_id)

        robot.status = "FAILED"
        robot.path = []
        robot.wait_steps = 0
        robot.waiting_for = None
        robot.failure_reason = reason
        robot.last_position = old_position
        self.failed_robot_positions[robot_id] = old_position

        self._isolate_failed_robot_from_mesh(robot_id)

        print("\n" + "=" * 70)
        print("[ROBOT FAILURE]")
        print(f"{robot_id} failed at {old_position}")
        print(f"[FAILURE REASON] {reason}")
        print("[RECOVERY] Released failed robot reservations")
        print("=" * 70)

        affected = []
        for other_id, other in self.robots.items():
            if other_id == robot_id:
                continue
            if other.destination is None:
                continue
            if other.status in ("ARRIVED", "FAILED", "NO_PATH"):
                continue

            if old_position in other.path:
                affected.append(other_id)

        for other_id in affected:
            print(
                f"[RECOVERY] {other_id} route affected by failed {robot_id}"
            )
            self.reroute_robot(
                other_id,
                reason=f"failure of {robot_id}",
                force=True,
            )

        self.update_mesh()
        return True

    # ========================================================
    # HELPERS
    # ========================================================

    def _isolate_failed_robot_from_mesh(self, robot_id):
        remove_method = getattr(self.mesh, "remove_robot", None)
        if callable(remove_method):
            remove_method(robot_id)
            return

        if hasattr(self.mesh, "positions"):
            self.mesh.positions.pop(robot_id, None)
        if hasattr(self.mesh, "neighbors"):
            self.mesh.neighbors.pop(robot_id, None)
            for neighbor_set in self.mesh.neighbors.values():
                neighbor_set.discard(robot_id)
        if hasattr(self.mesh, "states"):
            self.mesh.states.pop(robot_id, None)

    def get_blocked_positions(self, robot_id):
        """Static physical blockers for A* replanning."""
        blocked = set()
        this_robot = self.robots.get(robot_id)

        for other_id, other in self.robots.items():
            if other_id == robot_id:
                continue
            
            # FAILED robots are permanent brick walls.
            if other.status == "FAILED":
                blocked.add(other.position)
                
            # Avoid active robots we explicitly yielded to, preventing immediate traffic deadlocks.
            elif this_robot and this_robot.waiting_for == other_id:
                blocked.add(other.position)
                if other.path:
                    for cell in other.path:
                        blocked.add(cell)

        return blocked

    def get_active_neighbors(self, robot_id):
        return [
            other_id
            for other_id in self.mesh.get_neighbors(robot_id)
            if other_id in self.robots
            and self.robots[other_id].status != "FAILED"
        ]

    def get_current_occupants(self, position, exclude_id=None):
        occupants = []
        for robot_id, robot in self.robots.items():
            if robot_id == exclude_id:
                continue
            if robot.position == position:
                occupants.append(robot_id)
        return occupants

    def _priority_key(self, robot):
        return (-robot.priority, robot.wait_steps * -1, robot.robot_id)

    def higher_priority(self, robot_a, robot_b):
        if robot_a.priority != robot_b.priority:
            return robot_a.priority > robot_b.priority
        if robot_a.wait_steps != robot_b.wait_steps:
            return robot_a.wait_steps > robot_b.wait_steps
        return robot_a.robot_id < robot_b.robot_id

    def _path_is_real_movement(self, robot):
        if not robot.path:
            return False
        return any(cell != robot.position for cell in robot.path)

    # ========================================================
    # PATH PLANNING
    # ========================================================

    def plan_robot(self, robot_id, force=False):
        robot = self.robots[robot_id]

        if robot.destination is None:
            robot.status = "IDLE"
            robot.path = []
            return False

        if robot.position == robot.destination:
            robot.status = "ARRIVED"
            robot.path = []
            self.reservations.clear_robot(robot_id)
            return True

        self.reservations.clear_robot(robot_id)

        parked_cells = {
            other.position
            for other_id, other in self.robots.items()
            if other_id != robot_id
            and other.status == "FAILED"
        }

        if robot.destination in parked_cells:
            robot.path = []
            robot.status = "WAITING"
            robot.waiting_for = next(
                (other_id for other_id, other in self.robots.items()
                 if other_id != robot_id
                 and other.status == "FAILED"
                 and other.position == robot.destination),
                None,
            )
            return False

        normal_path = self.planner.find_path(
            robot.position,
            robot.destination,
            blocked_cells=parked_cells,
        )

        if normal_path is None:
            robot.status = "WAITING"
            robot.path = []
            return False

        safe_path = self.make_reservation_safe_path(
            robot_id,
            robot.position,
            normal_path,
            max_wait=self.max_wait_steps,
        )

        if safe_path is None:
            robot.path = normal_path
            robot.status = "WAITING"
            return False

        robot.path = safe_path
        robot.status = "MOVING"
        robot.waiting_for = None

        self.reservations.reserve_path(
            robot_id,
            robot.path,
            self.time_step,
        )
        return True

    def make_reservation_safe_path(
        self,
        robot_id,
        start_position,
        normal_path,
        max_wait=8,
    ):
        safe_path = []
        current_position = start_position
        relative_time = 0

        for desired_position in normal_path:
            wait_count = 0

            while not self.reservations.can_move(
                robot_id,
                self.time_step + relative_time,
                current_position,
                desired_position,
            ):
                conflict = self.reservations.get_move_conflict(
                    robot_id,
                    self.time_step + relative_time,
                    current_position,
                    desired_position,
                )

                if conflict:
                    pass

                safe_path.append(current_position)
                relative_time += 1
                wait_count += 1

                if wait_count > max_wait:
                    return None

            safe_path.append(desired_position)
            relative_time += 1
            current_position = desired_position

        return safe_path

    def reroute_robot(self, robot_id, reason="conflict", force=False):
        robot = self.robots[robot_id]

        if robot.destination is None or robot.status == "ARRIVED":
            return False

        robot.reroute_count += 1
        self.reservations.clear_robot(robot_id)

        parked_cells = {
            other.position
            for other_id, other in self.robots.items()
            if other_id != robot_id
            and other.status == "FAILED"
        }

        if robot.destination in parked_cells:
            robot.status = "WAITING"
            robot.waiting_for = next(
                (other_id for other_id, other in self.robots.items()
                 if other_id != robot_id
                 and other.status == "FAILED"
                 and other.position == robot.destination),
                None,
            )
            robot.path = []
            return False

        new_path = self.planner.find_path(
            robot.position,
            robot.destination,
            blocked_cells=parked_cells,
        )

        if new_path is None:
            robot.status = "WAITING"
            robot.path = []
            return False

        safe_path = self.make_reservation_safe_path(
            robot_id,
            robot.position,
            new_path,
            max_wait=(self.max_wait_steps if not force else 1),
        )

        if safe_path is None:
            robot.path = new_path
            robot.status = "WAITING"
            return False

        old_signature = tuple(robot.path or [])
        new_signature = tuple(safe_path)

        robot.path = safe_path
        robot.status = "MOVING"
        robot.wait_steps = 0
        robot.waiting_for = None

        self.reservations.reserve_path(
            robot_id,
            robot.path,
            self.time_step,
        )

        if old_signature == new_signature and not force:
            pass
        else:
            print(f"[REROUTE SUCCESS] {robot_id}: {robot.path[:3]}...")

        return True

    # ========================================================
    # PROPOSAL / CONFLICT DETECTION
    # ========================================================

    def get_next_position(self, robot_id):
        robot = self.robots[robot_id]
        if robot.status not in ("MOVING", "WAITING") or not robot.path:
            return robot.position
        return robot.path[0]

    def build_proposals(self):
        proposals = {}

        for robot_id, robot in self.robots.items():
            if (
                robot.status in ("ARRIVED", "FAILED", "NO_PATH")
                or robot.destination is None
                or robot.position == robot.destination
            ):
                continue

            if not robot.path:
                continue

            next_position = robot.path[0]
            proposals[robot_id] = {
                "from": robot.position,
                "to": next_position,
            }

        return proposals

    def replan_active_robots(self):
        active_ids = [
            rid
            for rid, robot in self.robots.items()
            if (
                robot.status not in ("ARRIVED", "FAILED", "NO_PATH")
                and robot.destination is not None
                and robot.position != robot.destination
            )
        ]

        active_ids.sort(
            key=lambda rid: (
                -self.robots[rid].priority,
                -self.robots[rid].wait_steps,
                rid,
            )
        )

        for robot_id in active_ids:
            self.reservations.clear_robot(robot_id)

        for robot_id in active_ids:
            robot = self.robots[robot_id]

            blocked = self.get_blocked_positions(robot_id)
            blocked.discard(robot.position)
            blocked.discard(robot.destination)

            normal_path = self.planner.find_path(
                robot.position,
                robot.destination,
                blocked_cells=blocked,
            )

            if normal_path is None:
                robot.status = "WAITING"
                robot.path = []
                robot.wait_steps += 1
                continue

            safe_path = self.make_reservation_safe_path(
                robot_id,
                robot.position,
                normal_path,
                max_wait=self.max_wait_steps,
            )

            if safe_path is None:
                robot.path = []
                robot.status = "WAITING"
                robot.wait_steps += 1
                continue

            robot.path = safe_path
            robot.status = "MOVING"
            robot.waiting_for = None

            self.reservations.reserve_path(
                robot_id,
                robot.path,
                self.time_step,
            )

        return active_ids

    def choose_winner(self, robot_ids):
        candidates = [self.robots[rid] for rid in robot_ids]
        winner = candidates[0]

        for candidate in candidates[1:]:
            if self.higher_priority(candidate, winner):
                winner = candidate

        return winner.robot_id

    def resolve_target_conflicts(self, proposals):
        changed = True

        while changed:
            changed = False
            target_groups = {}

            for robot_id, move in proposals.items():
                if move["to"] == move["from"]:
                    continue
                target_groups.setdefault(move["to"], []).append(robot_id)

            for target, robot_ids in target_groups.items():
                if len(robot_ids) <= 1:
                    continue

                winner_id = self.choose_winner(robot_ids)
                
                for loser_id in robot_ids:
                    if loser_id == winner_id:
                        continue
                    
                    self.make_robot_yield(loser_id, winner_id, "same target cell")
                    proposals[loser_id]["to"] = self.robots[loser_id].position
                    changed = True

        return proposals

    def resolve_swap_conflicts(self, proposals):
        checked = set()

        ids = list(proposals.keys())
        for i, robot_a in enumerate(ids):
            for robot_b in ids[i + 1:]:
                pair = tuple(sorted((robot_a, robot_b)))
                if pair in checked:
                    continue
                checked.add(pair)

                a = proposals[robot_a]
                b = proposals[robot_b]

                if (
                    a["from"] == b["to"]
                    and a["to"] == b["from"]
                    and a["from"] != a["to"]
                ):
                    winner_id = self.choose_winner([robot_a, robot_b])
                    loser_id = robot_b if winner_id == robot_a else robot_a

                    self.make_robot_yield(
                        loser_id,
                        winner_id,
                        "head-on swap conflict",
                    )
                    proposals[loser_id]["to"] = self.robots[loser_id].position

        return proposals

    def resolve_reservation_conflicts(self, proposals):
        for robot_id, move in proposals.items():
            if move["to"] == move["from"]:
                continue

            conflict = self.reservations.get_move_conflict(
                robot_id,
                self.time_step,
                move["from"],
                move["to"],
            )

            if conflict is None:
                continue

            other_id = conflict.get("robot_id")
            if other_id not in self.robots:
                continue

            winner_id = self.choose_winner([robot_id, other_id])
            loser_id = other_id if winner_id == robot_id else robot_id

            if loser_id == robot_id:
                self.make_robot_yield(
                    robot_id,
                    winner_id,
                    "reservation conflict",
                )
                proposals[robot_id]["to"] = self.robots[robot_id].position

        return proposals

    def resolve_physical_conflicts(self, proposals):
        for robot_id, move in proposals.items():
            if move["to"] == move["from"]:
                continue

            occupants = self.get_current_occupants(
                move["to"],
                exclude_id=robot_id,
            )

            if not occupants:
                continue

            failed_occupants = [
                other_id
                for other_id in occupants
                if self.robots[other_id].status == "FAILED"
            ]

            if failed_occupants:
                self.reroute_robot(
                    robot_id,
                    reason=f"failed robot occupying {move['to']}",
                    force=True,
                )
                proposals[robot_id]["to"] = self.robots[robot_id].position
                continue

            # Identify if all occupants are pushable (IDLE, ARRIVED, or Lower-Priority WAITING)
            pushable_occupants = [
                other_id
                for other_id in occupants
                if self.robots[other_id].status in ("IDLE", "ARRIVED")
                or (self.robots[other_id].status == "WAITING" and self.higher_priority(self.robots[robot_id], self.robots[other_id]))
            ]
            
            # If everyone in the cell is pushable, let the proposal pass to execute_proposals!
            if len(pushable_occupants) == len(occupants):
                continue

            winner_id = self.choose_winner([robot_id] + [o for o in occupants if o not in pushable_occupants])

            if winner_id != robot_id:
                self.make_robot_yield(
                    robot_id,
                    winner_id,
                    "physical occupancy",
                )
                proposals[robot_id]["to"] = self.robots[robot_id].position

        return proposals

    # ========================================================
    # YIELD / WAITING
    # ========================================================

    def make_robot_yield(self, robot_id, waiting_for, reason):
        robot = self.robots[robot_id]
        robot.status = "WAITING"
        robot.wait_steps += 1
        robot.waiting_for = waiting_for

        self.reservations.clear_robot(robot_id)

        if not robot.path:
            robot.path = []

    def retry_waiting_robots(self):
        waiting_ids = [
            rid
            for rid, robot in self.robots.items()
            if robot.status == "WAITING" and robot.destination is not None
        ]

        waiting_ids.sort(
            key=lambda rid: (
                -self.robots[rid].priority,
                -self.robots[rid].wait_steps,
                rid,
            )
        )

        for robot_id in waiting_ids:
            robot = self.robots[robot_id]
            robot.wait_steps += 1

            if robot.position == robot.destination:
                robot.status = "ARRIVED"
                robot.path = []
                continue

            if robot.wait_steps >= 2:
                self.plan_robot(robot_id)
                robot.waiting_for = None

    # ========================================================
    # DEADLOCK RECOVERY
    # ========================================================

    def detect_deadlock(self):
        active = [
            robot for robot in self.robots.values()
            if robot.status in ("MOVING", "WAITING")
            and robot.destination is not None
            and robot.position != robot.destination
        ]

        if len(active) <= 1:
            return False

        if self.no_progress_steps < self.deadlock_threshold:
            return False

        return True

    def recover_deadlock(self):
        active = [
            robot for robot in self.robots.values()
            if robot.status in ("MOVING", "WAITING")
            and robot.destination is not None
            and robot.position != robot.destination
        ]

        if not active:
            self.no_progress_steps = 0
            return

        winner = active[0]
        for candidate in active[1:]:
            if self.higher_priority(candidate, winner):
                winner = candidate

        print("\n" + "=" * 70)
        print("[DEADLOCK RECOVERY]")
        print(
            f"No movement for {self.no_progress_steps} steps. "
            f"{winner.robot_id} (priority={winner.priority}) gets recovery turn."
        )
        print("=" * 70)

        recovery_blocked_cells = self.get_blocked_positions(winner.robot_id)

        for robot in active:
            if robot.robot_id == winner.robot_id:
                continue
                
            recovery_blocked_cells.add(robot.position)

            if self.higher_priority(winner, robot):
                self.reservations.clear_robot(robot.robot_id)
                robot.status = "WAITING"
                robot.wait_steps += 1
                robot.waiting_for = winner.robot_id

        self.reservations.clear_robot(winner.robot_id)
        winner.wait_steps = 0
        winner.waiting_for = None

        normal_path = self.planner.find_path(
            winner.position,
            winner.destination,
            blocked_cells=recovery_blocked_cells,
        )

        if normal_path is not None:
            winner.path = normal_path
            winner.status = "MOVING"
            self.reservations.reserve_path(
                winner.robot_id,
                winner.path,
                self.time_step,
            )
            print(
                f"[DEADLOCK BREAKER] {winner.robot_id} resumes on "
                f"{winner.path[:6]}..."
            )
        else:
            print(f"[DEADLOCK BREAKER] {winner.robot_id} could not find a path around obstacles!")

        self.no_progress_steps = 0

    # ========================================================
    # SIMULTANEOUS MOVEMENT WITH PRIORITY SHOVING
    # ========================================================

    def execute_proposals(self, proposals):
        old_positions = {
            robot_id: robot.position
            for robot_id, robot in self.robots.items()
        }

        occupied_by = {}
        for robot_id, position in old_positions.items():
            occupied_by.setdefault(position, []).append(robot_id)

        accepted = {}
        claimed_targets = set()
        idle_pushes = {}

        for robot_id, move in proposals.items():
            robot = self.robots[robot_id]

            if robot.status != "MOVING":
                continue

            source = old_positions[robot_id]
            target = move["to"]

            if target == source:
                continue

            occupants = [
                other_id
                for other_id in occupied_by.get(target, [])
                if other_id != robot_id
            ]

            can_move = True
            push_candidates = []
            
            if occupants:
                all_pushable = True
                for occ in occupants:
                    occ_robot = self.robots[occ]
                    # Allowed to push IDLE, ARRIVED, or Lower-Priority WAITING robots.
                    if occ_robot.status in ("IDLE", "ARRIVED"):
                        continue
                    if occ_robot.status == "WAITING" and self.higher_priority(robot, occ_robot):
                        continue
                    all_pushable = False
                    break
                
                if all_pushable:
                    push_candidates = occupants
                else:
                    can_move = False

            if not can_move:
                continue

            if target in claimed_targets:
                continue

            is_swap = False
            for other_id, other_move in proposals.items():
                if other_id == robot_id:
                    continue

                other_source = old_positions.get(other_id)
                other_target = other_move.get("to")

                if (
                    other_source is not None
                    and other_target is not None
                    and target == other_source
                    and source == other_target
                    and target != source
                ):
                    is_swap = True
                    break

            if is_swap:
                continue

            accepted[robot_id] = target
            claimed_targets.add(target)
            
            for occ in push_candidates:
                idle_pushes[occ] = source

        # 1. Apply active moves
        for robot_id, target in accepted.items():
            robot = self.robots[robot_id]

            robot.position = target
            robot.battery = max(
                0.0,
                robot.battery - 0.1,
            )

            if robot.path:
                robot.path.pop(0)

            robot.wait_steps = 0
            robot.waiting_for = None

        # 2. Apply shoving physics
        for occ_id, push_target in idle_pushes.items():
            if occ_id not in accepted:
                self.robots[occ_id].position = push_target

        for robot_id in accepted.keys() | idle_pushes.keys():
            self.mesh.update_position(
                robot_id,
                self.robots[robot_id].position,
            )

        for robot_id in accepted:
            robot = self.robots[robot_id]

            if (
                robot.destination is not None
                and robot.position == robot.destination
            ):
                robot.status = "ARRIVED"
                robot.path = []
                self.reservations.clear_robot(robot_id)

        return bool(accepted), list(accepted)

    # ========================================================
    # MESH
    # ========================================================

    def update_mesh(self):
        self.mesh.rebuild_mesh()

        for robot_id, robot in self.robots.items():
            if robot.status == "FAILED":
                self._isolate_failed_robot_from_mesh(robot_id)

        for robot in self.robots.values():
            if robot.status == "FAILED":
                continue

            next_position = self.get_next_position(robot.robot_id)
            if robot.status in ("ARRIVED", "IDLE", "NO_PATH"):
                next_position = None

            state = {
                "robot_id": robot.robot_id,
                "position": robot.position,
                "destination": robot.destination,
                "status": robot.status,
                "battery": robot.battery,
                "priority": robot.priority,
                "next_position": next_position,
            }

            self.mesh.broadcast_state(robot.robot_id, state)

    # ========================================================
    # SIMULATION STEP
    # ========================================================

    def step(self):
        self.time_step += 1

        self.reservations.remove_expired(
            self.time_step
        )

        self.check_heartbeats()
        self.update_mesh()

        self.replan_active_robots()

        proposals = self.build_proposals()

        proposals = self.resolve_reservation_conflicts(
            proposals
        )
        proposals = self.resolve_target_conflicts(
            proposals
        )
        proposals = self.resolve_swap_conflicts(
            proposals
        )
        proposals = self.resolve_physical_conflicts(
            proposals
        )

        moved, moved_ids = self.execute_proposals(
            proposals
        )

        if moved:
            self.no_progress_steps = 0
        else:
            active = [
                robot
                for robot in self.robots.values()
                if robot.status in ("MOVING", "WAITING")
                and robot.destination is not None
                and robot.position != robot.destination
            ]

            self.no_progress_steps += 1 if active else 0

        if self.detect_deadlock():
            self.recover_deadlock()

        self.update_mesh()

        return moved_ids

    # ========================================================
    # V4.3 MULTI-HOP MESH API
    # ========================================================

    def find_mesh_route(self, source_id, destination_id):
        return self.mesh.find_route(
            source_id,
            destination_id,
        )

    def send_mesh_message(self, source_id, destination_id, message):
        return self.mesh.send_message(
            source_id,
            destination_id,
            message,
        )

    def mesh_connected(self, source_id, destination_id):
        return self.mesh.is_connected(
            source_id,
            destination_id,
        )

    # ========================================================
    # STATUS / RESULT
    # ========================================================

    def print_status(self):
        print(
            f"\n========== TIME {self.time_step} =========="
        )

        for robot in self.robots.values():
            next_position = (
                self.get_next_position(robot.robot_id)
                if robot.status not in ("ARRIVED", "FAILED", "NO_PATH")
                else None
            )

            print(
                f"{robot.robot_id}: "
                f"pos={robot.position} | "
                f"goal={robot.destination} | "
                f"priority={robot.priority} | "
                f"status={robot.status} | "
                f"next={next_position} | "
                f"wait={robot.wait_steps} | "
                f"waiting_for={robot.waiting_for}"
                + (
                    f" | failure_reason={robot.failure_reason}"
                    if robot.status == "FAILED"
                    else ""
                )
            )

        print("\nMESH")
        for robot_id in self.robots:
            print(
                f"{robot_id} -> "
                f"{self.get_active_neighbors(robot_id)}"
            )

        print("\nRESERVATIONS")
        self.reservations.print_reservations()
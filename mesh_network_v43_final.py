from collections import deque
from math import dist


class MeshNetwork:
    def __init__(self, communication_range=20):
        if communication_range <= 0:
            raise ValueError("communication_range must be greater than 0")
        self.communication_range = communication_range
        self.positions = {}
        self.neighbors = {}
        self.states = {}
        self.message_log = []

    def add_robot(self, robot_id, position):
        self.positions[robot_id] = position
        self.neighbors.setdefault(robot_id, set())

    def update_position(self, robot_id, position):
        if robot_id not in self.positions:
            return False
        self.positions[robot_id] = position
        return True

    def remove_robot(self, robot_id):
        if robot_id not in self.positions:
            return False
        self.positions.pop(robot_id, None)
        self.neighbors.pop(robot_id, None)
        self.states.pop(robot_id, None)
        for neighbor_set in self.neighbors.values():
            neighbor_set.discard(robot_id)
        return True

    def rebuild_mesh(self):
        self.neighbors = {robot_id: set() for robot_id in self.positions}
        robot_ids = list(self.positions)
        for i, robot_id in enumerate(robot_ids):
            for other_id in robot_ids[i + 1:]:
                if dist(self.positions[robot_id], self.positions[other_id]) <= self.communication_range:
                    self.neighbors[robot_id].add(other_id)
                    self.neighbors[other_id].add(robot_id)

    def broadcast_state(self, robot_id, state):
        if robot_id not in self.positions:
            return False
        self.states[robot_id] = state
        return True

    def get_neighbors(self, robot_id):
        return sorted(self.neighbors.get(robot_id, set()))

    def find_route(self, source_id, destination_id):
        if source_id not in self.positions or destination_id not in self.positions:
            return None
        if source_id == destination_id:
            return [source_id]
        queue = deque([source_id])
        parent = {source_id: None}
        while queue:
            current = queue.popleft()
            for neighbor in sorted(self.neighbors.get(current, set())):
                if neighbor in parent:
                    continue
                parent[neighbor] = current
                if neighbor == destination_id:
                    route = []
                    node = destination_id
                    while node is not None:
                        route.append(node)
                        node = parent[node]
                    return list(reversed(route))
                queue.append(neighbor)
        return None

    def is_connected(self, source_id, destination_id):
        return self.find_route(source_id, destination_id) is not None

    def send_message(self, source_id, destination_id, message):
        route = self.find_route(source_id, destination_id)
        if route is None:
            return None
        result = {
            "source": source_id,
            "destination": destination_id,
            "route": route,
            "hops": len(route) - 1,
            "message": message,
        }
        self.message_log.append(result)
        return result

from __future__ import annotations

import heapq
import math
from collections import defaultdict
from typing import Dict, List, Optional, Tuple, Iterable, Set


Grid = List[List[bool]]
Coord = Tuple[int, int]


class AStarPlanner:
    def __init__(self, grid: Grid) -> None:
        self.grid = grid
        self.rows = len(grid)
        self.cols = len(grid[0]) if self.rows > 0 else 0

    def heuristic(self, a: Coord, b: Coord) -> float:
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def in_bounds(self, node: Coord) -> bool:
        r, c = node
        return 0 <= r < self.rows and 0 <= c < self.cols

    def passable(self, node: Coord) -> bool:
        r, c = node
        return not self.grid[r][c]

    def neighbors(self, node: Coord) -> Iterable[Coord]:
        r, c = node
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        for dr, dc in directions:
            nr, nc = r + dr, c + dc
            if 0 <= nr < self.rows and 0 <= nc < self.cols and not self.grid[nr][nc]:
                yield (nr, nc)

    def plan(self, start: Coord, goal: Coord) -> List[Coord]:
        if not self.in_bounds(start) or not self.in_bounds(goal):
            raise ValueError("Start or goal is out of bounds")
        if not self.passable(start) or not self.passable(goal):
            raise ValueError("Start or goal is on an obstacle")

        frontier: List[Tuple[float, Coord]] = []
        heapq.heappush(frontier, (0.0, start))
        came_from: Dict[Coord, Optional[Coord]] = {start: None}
        cost_so_far: Dict[Coord, float] = {start: 0.0}

        while frontier:
            _, current = heapq.heappop(frontier)
            if current == goal:
                break
            for next_node in self.neighbors(current):
                new_cost = cost_so_far[current] + 1.0
                if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                    cost_so_far[next_node] = new_cost
                    priority = new_cost + self.heuristic(next_node, goal)
                    heapq.heappush(frontier, (priority, next_node))
                    came_from[next_node] = current

        if goal not in came_from:
            raise ValueError("No path found between %s and %s" % (start, goal))

        path = []
        node = goal
        while node is not None:
            path.append(node)
            node = came_from.get(node)
        path.reverse()
        return path


class DStarLitePlanner:
    INF = 10 ** 9

    def __init__(self, grid: Grid) -> None:
        self.grid = grid
        self.rows = len(grid)
        self.cols = len(grid[0]) if self.rows > 0 else 0
        self.g: Dict[Coord, float] = defaultdict(lambda: DStarLitePlanner.INF)
        self.rhs: Dict[Coord, float] = defaultdict(lambda: DStarLitePlanner.INF)
        self.open: List[Tuple[Tuple[float, float], Coord]] = []
        self.last_start: Optional[Coord] = None
        self.start: Optional[Coord] = None
        self.goal: Optional[Coord] = None
        self.km = 0.0
        self.obstacles: Set[Coord] = set()

    def heuristic(self, a: Coord, b: Coord) -> float:
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def in_bounds(self, node: Coord) -> bool:
        r, c = node
        return 0 <= r < self.rows and 0 <= c < self.cols

    def passable(self, node: Coord) -> bool:
        r, c = node
        return not self.grid[r][c]

    def neighbors(self, node: Coord) -> Iterable[Coord]:
        r, c = node
        for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            nr, nc = r + dr, c + dc
            if 0 <= nr < self.rows and 0 <= nc < self.cols and not self.grid[nr][nc]:
                yield (nr, nc)

    def initialize(self, start: Coord, goal: Coord) -> None:
        if not self.in_bounds(start) or not self.in_bounds(goal):
            raise ValueError("Start or goal is out of bounds")
        if not self.passable(start) or not self.passable(goal):
            raise ValueError("Start or goal is on an obstacle")

        self.start = start
        self.goal = goal
        self.last_start = start
        self.km = 0.0
        self.g.clear()
        self.rhs.clear()
        self.open.clear()
        self.rhs[goal] = 0.0
        key = self.calculate_key(goal)
        heapq.heappush(self.open, (key, goal))
        self.obstacles = { (r, c) for r in range(self.rows) for c in range(self.cols) if self.grid[r][c] }
        self.compute_shortest_path()

    def calculate_key(self, node: Coord) -> Tuple[float, float]:
        g_rhs = min(self.g[node], self.rhs[node])
        h_val = 0.0 if self.start is None else self.heuristic(self.start, node)
        return (g_rhs + h_val + self.km, g_rhs)

    def update_vertex(self, u: Coord) -> None:
        if u != self.goal:
            min_rhs = DStarLitePlanner.INF
            for succ in self.neighbors(u):
                cost = 1.0
                val = self.g[succ] + cost
                if val < min_rhs:
                    min_rhs = val
            self.rhs[u] = min_rhs
        if self.g[u] != self.rhs[u]:
            heapq.heappush(self.open, (self.calculate_key(u), u))

    def compute_shortest_path(self) -> None:
        while self.open:
            (k_old, u) = self.open[0]
            k_new = self.calculate_key(u)
            if k_old > k_new:
                heapq.heappop(self.open)
                heapq.heappush(self.open, (k_new, u))
                continue
            if (k_old >= self.calculate_key(self.start) and 
                self.rhs[self.start] == self.g[self.start]):
                break
            heapq.heappop(self.open)
            if self.g[u] > self.rhs[u]:
                self.g[u] = self.rhs[u]
                for pred in self.predecessors(u):
                    self.update_vertex(pred)
            else:
                self.g[u] = DStarLitePlanner.INF
                self.update_vertex(u)
                for pred in self.predecessors(u):
                    self.update_vertex(pred)

    def predecessors(self, node: Coord) -> Iterable[Coord]:
        return self.neighbors(node)

    def successors(self, node: Coord) -> Iterable[Coord]:
        return self.neighbors(node)

    def update_obstacles(self, cells: Iterable[Coord]) -> None:
        changed = False
        for cell in cells:
            if cell not in self.obstacles:
                r, c = cell
                if 0 <= r < self.rows and 0 <= c < self.cols:
                    self.grid[r][c] = True
                    self.obstacles.add(cell)
                    self.update_vertex(cell)
                    changed = True
        if changed:
            self.compute_shortest_path()

    def move_start(self, new_start: Coord) -> None:
        if self.start is None:
            raise ValueError("Planner not initialised")
        self.km += self.heuristic(self.last_start, new_start)
        self.last_start = new_start
        self.start = new_start
        self.update_vertex(new_start)
        self.compute_shortest_path()

    def get_path(self) -> List[Coord]:
        if self.start is None or self.goal is None:
            raise ValueError("Planner not initialised")
        if self.rhs[self.start] == DStarLitePlanner.INF:
            raise ValueError("No path exists")
        path = [self.start]
        current = self.start
        step_limit = self.rows * self.cols
        while current != self.goal and step_limit > 0:
            step_limit -= 1
            min_node: Optional[Coord] = None
            min_cost = DStarLitePlanner.INF
            for succ in self.successors(current):
                cost = 1.0 + self.g[succ]
                if cost < min_cost:
                    min_cost = cost
                    min_node = succ
            if min_node is None or self.g[min_node] == DStarLitePlanner.INF:
                raise ValueError("No path exists")
            path.append(min_node)
            current = min_node
        if current != self.goal:
            raise ValueError("Failed to reach goal within step limit")
        return path

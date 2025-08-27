import random
from .uav import UAV
from .path_planner import AStarPlanner
from .threat_simulator import ThreatSimulator

class Coordinator:
    def __init__(self, rows, cols, start_goals, use_dstar=False, seed=None):
        self.rows = rows
        self.cols = cols
        self.use_dstar = use_dstar
        self.grid = [[False] * cols for _ in range(rows)]
        self.uavs: list[UAV] = []
        self.rng = random.Random(seed)

        for idx, (s, g) in enumerate(start_goals):
            self.uavs.append(UAV(idx, s, g, self.grid, use_dstar))

        self.threat_sim = ThreatSimulator(rows, cols, rng_seed=seed)

    def step(self):
        self.threat_sim.step()
        self.apply_threats()

        # Collision avoidance
        occupied = set()
        for u in self.uavs:
            if u.path:
                nxt = u.path[0]
                if nxt in occupied:
                    u.wait_steps = getattr(u, "wait_steps", 0) + 1
                    print(f"[Collision Avoidance] UAV {u.id} waiting")
                else:
                    occupied.add(nxt)

        # Move UAVs
        for u in self.uavs:
            if getattr(u, "wait_steps", 0) > 0:
                u.wait_steps -= 1
                continue
            u.step()

        self.check_goal_swapping()

    def apply_threats(self):
        self.grid = [[False] * self.cols for _ in range(self.rows)]
        for t in self.threat_sim.threats:
            r0, c0, rad = t.row, t.col, t.radius
            for r in range(max(0, r0 - rad), min(self.rows, r0 + rad + 1)):
                for c in range(max(0, c0 - rad), min(self.cols, c0 + rad + 1)):
                    if (r - r0) ** 2 + (c - c0) ** 2 <= rad ** 2:
                        self.grid[r][c] = True

    def check_goal_swapping(self):
        if len(self.uavs) < 2:
            return
        u1, u2 = self.uavs[0], self.uavs[1]
        if u1.at_goal or u2.at_goal:
            return
        cost_current = len(u1.path) + len(u2.path)
        planner = AStarPlanner(self.grid)
        cost_swap = len(planner.plan(u1.pos, u2.goal)) + len(planner.plan(u2.pos, u1.goal))
        if cost_swap < cost_current or random.random() < 0.3:
            g1, g2 = u1.goal, u2.goal
            u1.set_goal(g2)
            u2.set_goal(g1)
            print(f"[Coordinator] Goals swapped between UAV {u1.id} and UAV {u2.id}")

    def all_uavs_at_goal(self):
        return all(u.at_goal for u in self.uavs)

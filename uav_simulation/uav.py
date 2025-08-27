class UAV:
    def __init__(self, id, start, goal, grid, use_dstar=False):
        self.id = id
        self.start = start
        self.goal = goal
        self.pos = start
        self.grid = grid
        self.at_goal = False
        self.use_dstar = use_dstar

        # pick planner
        if use_dstar:
            from .path_planner import DStarLitePlanner
            self.planner = DStarLitePlanner(grid)
        else:
            from .path_planner import AStarPlanner
            self.planner = AStarPlanner(grid)

        self.path = []
        self.initialised = False
        self.wait_steps = 0

    def plan_path(self):
        if self.use_dstar:
            # D* Lite needs initialization first
            if not self.initialised:
                self.planner.initialize(self.pos, self.goal)
                self.initialised = True
            self.path = self.planner.get_path()
        else:
            # A* just directly computes
            self.path = self.planner.plan(self.pos, self.goal)

    def step(self):
        if self.at_goal:
            return

        if not self.path:
            self.plan_path()
            if not self.path:
                return

        self.pos = self.path.pop(0)

        if self.pos == self.goal:
            self.at_goal = True

    def set_goal(self, new_goal):
        self.goal = new_goal
        self.at_goal = False
        self.initialised = False
        self.plan_path()

import matplotlib.pyplot as plt

class Visualizer:
    def __init__(self, rows, cols):
        self.rows = rows
        self.cols = cols
        self.fig, self.ax = plt.subplots(figsize=(6, 6))

    def render(self, grid, uavs, threats, step):
        self.ax.clear()
        arr = [[1 if cell else 0 for cell in row] for row in grid]
        self.ax.imshow(arr, cmap="Greys", origin="upper")

        # threats
        for t in threats:
            circ = plt.Circle((t.col, t.row), t.radius, color="red", alpha=0.4)
            self.ax.add_patch(circ)

        # UAVs and goals
        for u in uavs:
            if u.path:
                xs, ys = zip(*[(c, r) for r, c in u.path])
                self.ax.plot(xs, ys, "--", alpha=0.6)
            self.ax.scatter(u.pos[1], u.pos[0], marker="o", s=100, label=f"UAV {u.id}")
            self.ax.scatter(u.goal[1], u.goal[0], marker="*", s=120, label=f"Goal {u.id}")

        self.ax.set_title(f"Step {step} | Threats: {len(threats)}")
        self.ax.legend(loc="upper right", fontsize=7)
        plt.pause(0.01)

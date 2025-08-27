import random

class Threat:
    def __init__(self, row, col, radius, lifetime):
        self.row = row
        self.col = col
        self.radius = radius
        self.lifetime = lifetime

    def decay(self):
        self.lifetime -= 1
        return self.lifetime <= 0


class ThreatSimulator:
    def __init__(self,
                 rows: int,
                 cols: int,
                 spawn_prob: float = 0.25,
                 min_radius: int = 1,
                 max_radius: int = 3,
                 min_lifetime: int = 8,
                 max_lifetime: int = 20,
                 rng_seed: int | None = None):
        self.rows = rows
        self.cols = cols
        self.spawn_prob = spawn_prob
        self.min_radius = min_radius
        self.max_radius = max_radius
        self.min_lifetime = min_lifetime
        self.max_lifetime = max_lifetime
        self.rng = random.Random(rng_seed)
        self.threats: list[Threat] = []

    def step(self):
        # decay old threats
        self.threats = [t for t in self.threats if not t.decay()]

        # possibly spawn new threat
        if self.rng.random() < self.spawn_prob:
            r = self.rng.randint(0, self.rows - 1)
            c = self.rng.randint(0, self.cols - 1)
            rad = self.rng.randint(self.min_radius, self.max_radius)
            lifetime = self.rng.randint(self.min_lifetime, self.max_lifetime)
            self.threats.append(Threat(r, c, rad, lifetime))

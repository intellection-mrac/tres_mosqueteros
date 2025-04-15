from collections import defaultdict

class ZoneManager:
    def __init__(self, grid_size, zone_count):
        self.grid_size = grid_size
        self.zone_count = zone_count
        self.zone_size = grid_size // zone_count
        self.zone_coverage = {}
        self.zone_totals = {}
        self.completed_zones = set()
        self.current_zone = None
        self.next_zone = None
        self.exit_point = None

    def zone_index(self, x, y):
        return (x // self.zone_size, y // self.zone_size)

    def register_valid_points(self, valid_points):
        self.zone_coverage = defaultdict(int)
        self.zone_totals = defaultdict(int)
        for x, y in valid_points:
            zone = self.zone_index(x, y)
            self.zone_totals[zone] += 1

    def update_coverage(self, x, y):
        z = self.zone_index(x, y)
        self.zone_coverage[z] += 1
        if self.is_zone_complete(z):
            self.completed_zones.add(z)

    def is_zone_complete(self, zone):
        covered = self.zone_coverage.get(zone, 0)
        total = self.zone_totals.get(zone, 1)
        return covered / total >= 0.98

    def select_next_zone(self):
        candidates = [z for z in self.zone_totals if z not in self.completed_zones]
        if not candidates:
            return None
        return min(candidates, key=lambda z: self.zone_coverage.get(z, 0))

    def find_exit_point(self, valid_points, current_zone, target_zone):
        cx, cy = current_zone
        tx, ty = target_zone
        border = []
        for x, y in valid_points:
            z = self.zone_index(x, y)
            if z == current_zone:
                if abs(x - tx * self.zone_size) <= 1 or abs(y - ty * self.zone_size) <= 1:
                    border.append((x, y))
        if not border:
            return None
        return min(border, key=lambda p: abs(p[0] - tx * self.zone_size) + abs(p[1] - ty * self.zone_size))

    def enter_new_zone(self, current_position, valid_points):
        self.current_zone = self.zone_index(*current_position)
        self.next_zone = self.select_next_zone()
        if self.next_zone:
            self.exit_point = self.find_exit_point(valid_points, self.current_zone, self.next_zone)

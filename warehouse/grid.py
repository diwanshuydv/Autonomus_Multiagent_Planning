"""Warehouse grid map: static shelves plus time-bounded dynamic blockages."""
from collections import deque

INF = float("inf")


class Grid:
    """4-connected grid. Cells are (row, col) tuples.

    Static shelves never change. Dynamic blockages are intervals [t0, t1)
    during which a cell cannot be entered or occupied; they are added only
    when the disruption actually happens, so planners never see the future.
    """

    def __init__(self, height, width, shelves, stations=(), homes=(), pickups=()):
        self.H, self.W = height, width
        self.shelves = frozenset(shelves)
        self.stations = list(stations)
        self.homes = list(homes)
        self.pickups = list(pickups)
        self._nbrs = {}
        for r in range(height):
            for c in range(width):
                if (r, c) in self.shelves:
                    continue
                self._nbrs[(r, c)] = [
                    n for n in ((r - 1, c), (r + 1, c), (r, c - 1), (r, c + 1)) if self.free(n)
                ]
        self.blocks = {}  # cell -> list[(t0, t1)]
        self.max_block_t = 0
        self._dist = {}

    # ---- static structure -------------------------------------------------
    def free(self, cell):
        r, c = cell
        return 0 <= r < self.H and 0 <= c < self.W and cell not in self.shelves

    @property
    def cells(self):
        return list(self._nbrs)

    def neighbors(self, cell):
        return self._nbrs[cell]

    def dist_table(self, goal):
        """BFS distances to `goal` on the static map (admissible A* heuristic)."""
        table = self._dist.get(goal)
        if table is None:
            table = {goal: 0}
            q = deque([goal])
            while q:
                u = q.popleft()
                for v in self._nbrs[u]:
                    if v not in table:
                        table[v] = table[u] + 1
                        q.append(v)
            self._dist[goal] = table
        return table

    def dist(self, a, b):
        return self.dist_table(b).get(a, INF)

    # ---- dynamic blockages ------------------------------------------------
    def add_block(self, cell, t0, t1=INF):
        """Block `cell` during [t0, t1); t1 = INF makes the blockage permanent."""
        self.blocks.setdefault(cell, []).append((t0, t1))
        # a permanent blockage never changes after t0, so it does not extend
        # the time horizon after which the world is static
        self.max_block_t = max(self.max_block_t, t1 if t1 != INF else t0)

    def is_blocked(self, cell, t):
        iv = self.blocks.get(cell)
        if not iv:
            return False
        for t0, t1 in iv:
            if t0 <= t < t1:
                return True
        return False

    def blocked_after(self, cell, t):
        """True if the cell is blocked at some time >= t."""
        return any(t1 > t for _, t1 in self.blocks.get(cell, ()))

    def blocked_cells(self, t):
        return {c for c, iv in self.blocks.items() if any(t0 <= t < t1 for t0, t1 in iv)}

    def permanent_cells(self):
        """Cells that are blocked from some time on and never free up again."""
        return {c for c, iv in self.blocks.items() if any(t1 == INF for _, t1 in iv)}

    def reachable(self, start, obstacles):
        """Cells reachable from `start` on the static map without entering `obstacles`."""
        seen = {start}
        q = deque([start])
        while q:
            u = q.popleft()
            for v in self._nbrs[u]:
                if v not in seen and v not in obstacles:
                    seen.add(v)
                    q.append(v)
        return seen

    def expire(self, t):
        """Drop intervals that ended before t (planners only query t' >= t)."""
        for cell in list(self.blocks):
            iv = [(a, b) for a, b in self.blocks[cell] if b > t]
            if iv:
                self.blocks[cell] = iv
            else:
                del self.blocks[cell]


def make_warehouse(n_block_rows=4, n_block_cols=6, block_h=2, block_w=4,
                   row_aisle=2, col_aisle=1, margin_x=3, margin_y=2):
    """Build a typical warehouse layout.

    - Shelf blocks (block_h x block_w) separated by aisles.
    - Robot home/parking cells: the whole top and bottom rows (row 1 and
      row H-2 stay free as corridors, so parked robots never wall anything off).
    - Delivery stations: every second cell of the left and right columns.
    - Pickup cells: free cells beside a shelf.
    """
    H = 2 * margin_y + n_block_rows * block_h + (n_block_rows - 1) * row_aisle
    W = 2 * margin_x + n_block_cols * block_w + (n_block_cols - 1) * col_aisle
    shelves = set()
    for i in range(n_block_rows):
        for j in range(n_block_cols):
            r0 = margin_y + i * (block_h + row_aisle)
            c0 = margin_x + j * (block_w + col_aisle)
            for r in range(r0, r0 + block_h):
                for c in range(c0, c0 + block_w):
                    shelves.add((r, c))
    homes = [(0, c) for c in range(W)] + [(H - 1, c) for c in range(W)]
    stations = [(r, c) for c in (0, W - 1) for r in range(margin_y, H - margin_y, 2)]
    special = set(homes) | set(stations)
    pickups = []
    for r in range(H):
        for c in range(W):
            if (r, c) in shelves or (r, c) in special:
                continue
            if any(n in shelves for n in ((r - 1, c), (r + 1, c), (r, c - 1), (r, c + 1))):
                pickups.append((r, c))
    return Grid(H, W, shelves, stations, homes, pickups)

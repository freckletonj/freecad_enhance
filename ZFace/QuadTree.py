import math


##################################################
# QuadTree

# --- Simple Quadtree for 2D points with stored Z value ---
class QuadTree:
    """
    Lightweight quadtree that stores points (x,y,z).
    - Built from a bounding box [xmin,xmax,ymin,ymax]
    - insert(x,y,z)
    - nearest(x,y) -> (x,y,z,dist) or None
    - bulk construction from points list
    """

    class Node:
        def __init__(self, xmin, xmax, ymin, ymax, capacity=8):
            self.xmin = xmin; self.xmax = xmax; self.ymin = ymin; self.ymax = ymax
            self.capacity = capacity
            self.points = []  # list of (x,y,z)
            self.children = None  # [nw, ne, sw, se] or None

        def contains(self, x, y):
            return (self.xmin <= x <= self.xmax) and (self.ymin <= y <= self.ymax)

        def subdivide(self):
            mx = 0.5 * (self.xmin + self.xmax)
            my = 0.5 * (self.ymin + self.ymax)
            self.children = [
                QuadTree.Node(self.xmin, mx, my, self.ymax, self.capacity),  # NW
                QuadTree.Node(mx, self.xmax, my, self.ymax, self.capacity),  # NE
                QuadTree.Node(self.xmin, mx, self.ymin, my, self.capacity),  # SW
                QuadTree.Node(mx, self.xmax, self.ymin, my, self.capacity),  # SE
            ]

        def insert(self, x, y, z):
            if not self.contains(x, y):
                return False

            if self.children is None:
                if len(self.points) < self.capacity:
                    self.points.append((x, y, z))
                    return True
                else:
                    self.subdivide()
                    # move existing points into children
                    pts = self.points
                    self.points = []
                    for px, py, pz in pts:
                        inserted = False
                        for ch in self.children:
                            if ch.insert(px, py, pz):
                                inserted = True
                                break
                        if not inserted:
                            # This shouldn't happen, but if a point is exactly on boundary it might
                            self.points.append((px, py, pz))
                    # fallthrough to insert the new point into a child

            # insert into appropriate child
            if self.children is not None:
                for ch in self.children:
                    if ch.insert(x, y, z):
                        return True
                # if none matched due to numerical boundary issues, keep in this node
                self.points.append((x, y, z))
                return True

            return False

        def nearest(self, x, y, best):
            """
            best: tuple (best_dist_sq, (bx,by,bz)) or None
            returns best tuple
            """
            # compute minimal possible distance^2 from query point to this node's bbox
            dx = 0.0
            if x < self.xmin:
                dx = self.xmin - x
            elif x > self.xmax:
                dx = x - self.xmax
            dy = 0.0
            if y < self.ymin:
                dy = self.ymin - y
            elif y > self.ymax:
                dy = y - self.ymax
            min_dist_sq = dx*dx + dy*dy

            if best is not None and min_dist_sq >= best[0]:
                # subtree cannot contain a closer point
                return best

            # check points stored in this node
            for px, py, pz in self.points:
                ddx = px - x
                ddy = py - y
                d2 = ddx*ddx + ddy*ddy
                if best is None or d2 < best[0]:
                    best = (d2, (px, py, pz))

            # if no children, return
            if self.children is None:
                return best

            # order children by proximity to query point to improve pruning
            child_order = sorted(
                self.children,
                key=lambda ch: ((max(ch.xmin - x, 0, x - ch.xmax))**2 + (max(ch.ymin - y, 0, y - ch.ymax))**2)
            )
            for ch in child_order:
                best = ch.nearest(x, y, best)
            return best

    def __init__(self, xmin, xmax, ymin, ymax, capacity=8):
        # ensure valid bounds
        if xmax <= xmin:
            xmax = xmin + 1.0
        if ymax <= ymin:
            ymax = ymin + 1.0
        self.root = QuadTree.Node(xmin, xmax, ymin, ymax, capacity)

    def insert(self, x, y, z):
        return self.root.insert(x, y, z)

    def nearest(self, x, y):
        best = self.root.nearest(x, y, None)
        if best is None:
            return None
        d2, (px, py, pz) = best
        return (px, py, pz, math.sqrt(d2))

    @classmethod
    def from_point_list(cls, points, bounds_padding=0.0, capacity=8):
        """
        points: list of (x,y,z)
        bounds_padding: optional padding added around min/max
        """
        if not points:
            return None
        xs = [p[0] for p in points]; ys = [p[1] for p in points]
        xmin, xmax = min(xs), max(xs)
        ymin, ymax = min(ys), max(ys)
        dx = xmax - xmin; dy = ymax - ymin
        if dx == 0: dx = 1.0
        if dy == 0: dy = 1.0
        padx = bounds_padding if bounds_padding > 0 else 0.01 * dx
        pady = bounds_padding if bounds_padding > 0 else 0.01 * dy
        qt = cls(xmin - padx, xmax + padx, ymin - pady, ymax + pady, capacity)
        for x, y, z in points:
            qt.insert(x, y, z)
        return qt

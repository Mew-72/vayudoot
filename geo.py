"""Small geographic helpers (flat-earth is fine for a ~200 m field)."""
import math

M_PER_DEG = 111_320.0


class LocalFrame:
    """Local metres (x = east, y = north) around an origin lat/lon."""

    def __init__(self, lat0, lon0):
        self.lat0, self.lon0 = lat0, lon0
        self.kx = M_PER_DEG * math.cos(math.radians(lat0))
        self.ky = M_PER_DEG

    def to_xy(self, lat, lon):
        return (lon - self.lon0) * self.kx, (lat - self.lat0) * self.ky

    def to_ll(self, x, y):
        return self.lat0 + y / self.ky, self.lon0 + x / self.kx


def dist_m(lat1, lon1, lat2, lon2):
    dn = (lat2 - lat1) * M_PER_DEG
    de = (lon2 - lon1) * M_PER_DEG * math.cos(math.radians((lat1 + lat2) / 2))
    return math.hypot(dn, de)


def point_in_polygon(lat, lon, poly_ll):
    """Ray casting. poly_ll is a list of (lat, lon)."""
    inside = False
    n = len(poly_ll)
    for i in range(n):
        y1, x1 = poly_ll[i]
        y2, x2 = poly_ll[(i + 1) % n]
        if (y1 > lat) != (y2 > lat):
            x_cross = (x2 - x1) * (lat - y1) / (y2 - y1) + x1
            if lon < x_cross:
                inside = not inside
    return inside

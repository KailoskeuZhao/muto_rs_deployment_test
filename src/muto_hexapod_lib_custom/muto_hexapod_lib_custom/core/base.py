"""Small geometry types retained from Yahboom's Muto library."""


class point3d:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = x
        self.y = y
        self.z = z

    @classmethod
    def from_tuple(cls, values):
        return cls(values[0], values[1], values[2])

    def as_tuple(self):
        return (self.x, self.y, self.z)

    def __sub__(self, other):
        return point3d(self.x - other.x, self.y - other.y, self.z - other.z)

    def __add__(self, other):
        return point3d(self.x + other.x, self.y + other.y, self.z + other.z)

    def __mul__(self, scalar):
        return point3d(self.x * scalar, self.y * scalar, self.z * scalar)

    def __eq__(self, other):
        if not isinstance(other, point3d):
            return NotImplemented
        return self.x == other.x and self.y == other.y and self.z == other.z


class locations:
    def __init__(self, points=None):
        self._points = list(points or [point3d() for _ in range(6)])
        if len(self._points) != 6:
            raise ValueError('locations requires six points')

    @classmethod
    def from_list(cls, values):
        return cls([point3d.from_tuple(value) for value in values])

    def get(self, index):
        return self._points[index]

    def as_tuples(self):
        return tuple(point.as_tuple() for point in self._points)

    def __sub__(self, other):
        return locations([
            left - right for left, right in zip(self._points, other._points)
        ])

    def __add__(self, other):
        return locations([
            left + right for left, right in zip(self._points, other._points)
        ])

    def __mul__(self, scalar):
        return locations([point * scalar for point in self._points])

    def __str__(self):
        return str(self.as_tuples())

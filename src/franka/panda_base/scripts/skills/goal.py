class Goal():
    def __init__(self, x, y, z, dx, dy, dz, dw):
        self.x = x
        self.y = y
        self.z = z
        self.dx = dx
        self.dy = dy
        self.dz = dz
        self.dw = dw

    def copy(self):
        return Goal(self.x, self.y, self.z, self.dx, self.dy, self.dz, self.dw)
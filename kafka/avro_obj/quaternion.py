class Quaternion:
    def __init__(self, x, y, z, w) -> None:
        self.x = x
        self.y = y
        self.z = z
        self.w = w

    def avro(self):
        return {'x': self.x, 'y': self.y, 'z': self.z, 'w': self.w}
class Point3d:
    def __init__(self, x, y, z) -> None:
        self.x = x
        self.y = y
        self.z = z

    def avro(self):
        return {'x': self.x, 'y': self.y, 'z': self.z}
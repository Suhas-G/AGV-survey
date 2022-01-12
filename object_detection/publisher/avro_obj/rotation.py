class Rotation:
    def __init__(self, rotation):
        self.m00 = rotation[0, 0]
        self.m01 = rotation[0, 1]
        self.m02 = rotation[0, 2]
        self.m10 = rotation[1, 0]
        self.m11 = rotation[1, 1]
        self.m12 = rotation[1, 2]
        self.m20 = rotation[2, 0]
        self.m21 = rotation[2, 1]
        self.m22 = rotation[0, 2]

    def avro(self):
        return {key: self.__dict__[key] if key[0] == 'm'}
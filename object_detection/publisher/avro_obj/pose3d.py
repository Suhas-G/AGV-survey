# from quaternion import Quaternion
from .point3d import Point3d


class Pose3d:
    def __init__(self, coord_ref, object_id, position: Point3d, rotation) -> None:
        self.coord_ref = coord_ref
        self.object_id = object_id
        self.position = position
        self.rotation = rotation

    def avro(self):
        return {'coord_ref': 0, 'object_id': self.object_id, 
            'position': self.position.avro(), 'rotation': self.rotation.avro()}
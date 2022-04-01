# -*- coding: utf-8 -*-

""" avro python class for file: Pose3d """

import json
from .helpers import default_json_serialize, todict
from typing import Union
from .Point3d import Point3d
from .Quaternion import Quaternion


class Pose3d(object):

    schema = """
    {
        "type": "record",
        "name": "Pose3d",
        "fields": [
            {
                "name": "position",
                "type": {
                    "type": "record",
                    "name": "Point3d",
                    "fields": [
                        {
                            "name": "x",
                            "type": "float"
                        },
                        {
                            "name": "y",
                            "type": "float"
                        },
                        {
                            "name": "z",
                            "type": "float"
                        }
                    ],
                    "aliases": [
                        "Position3d"
                    ],
                    "namespace": "de.dfki.cos.mrk40.avro"
                }
            },
            {
                "name": "orientation",
                "type": {
                    "type": "record",
                    "name": "Quaternion",
                    "fields": [
                        {
                            "name": "x",
                            "type": "float"
                        },
                        {
                            "name": "y",
                            "type": "float"
                        },
                        {
                            "name": "z",
                            "type": "float"
                        },
                        {
                            "name": "w",
                            "type": "float"
                        }
                    ],
                    "aliases": [
                        "Rotation3d",
                        "Orientation3d"
                    ],
                    "namespace": "de.dfki.cos.mrk40.avro"
                }
            }
        ],
        "namespace": "de.dfki.cos.mrk40.avro"
    }
    """

    def __init__(self, obj: Union[str, dict, 'Pose3d']) -> None:
        if isinstance(obj, str):
            obj = json.loads(obj)

        elif isinstance(obj, type(self)):
            obj = obj.__dict__

        elif not isinstance(obj, dict):
            raise TypeError(
                f"{type(obj)} is not in ('str', 'dict', 'Pose3d')"
            )

        self.set_position(obj.get('position', None))

        self.set_orientation(obj.get('orientation', None))

    def dict(self):
        return todict(self)

    def set_position(self, values: Point3d) -> None:

        self.position = Point3d(values)

    def get_position(self) -> Point3d:

        return self.position

    def set_orientation(self, values: Quaternion) -> None:

        self.orientation = Quaternion(values)

    def get_orientation(self) -> Quaternion:

        return self.orientation

    def serialize(self) -> None:
        return json.dumps(self, default=default_json_serialize)

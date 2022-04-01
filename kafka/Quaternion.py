# -*- coding: utf-8 -*-

""" avro python class for file: Quaternion """

import json
from .helpers import default_json_serialize, todict
from typing import Union


class Quaternion(object):

    schema = """
    {
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
    """

    def __init__(self, obj: Union[str, dict, 'Quaternion']) -> None:
        if isinstance(obj, str):
            obj = json.loads(obj)

        elif isinstance(obj, type(self)):
            obj = obj.__dict__

        elif not isinstance(obj, dict):
            raise TypeError(
                f"{type(obj)} is not in ('str', 'dict', 'Quaternion')"
            )

        self.set_x(obj.get('x', None))

        self.set_y(obj.get('y', None))

        self.set_z(obj.get('z', None))

        self.set_w(obj.get('w', None))

    def dict(self):
        return todict(self)

    def set_x(self, value: float) -> None:

        if isinstance(value, float):
            self.x = value
        else:
            raise TypeError("field 'x' should be type float")

    def get_x(self) -> float:

        return self.x

    def set_y(self, value: float) -> None:

        if isinstance(value, float):
            self.y = value
        else:
            raise TypeError("field 'y' should be type float")

    def get_y(self) -> float:

        return self.y

    def set_z(self, value: float) -> None:

        if isinstance(value, float):
            self.z = value
        else:
            raise TypeError("field 'z' should be type float")

    def get_z(self) -> float:

        return self.z

    def set_w(self, value: float) -> None:

        if isinstance(value, float):
            self.w = value
        else:
            raise TypeError("field 'w' should be type float")

    def get_w(self) -> float:

        return self.w

    def serialize(self) -> None:
        return json.dumps(self, default=default_json_serialize)

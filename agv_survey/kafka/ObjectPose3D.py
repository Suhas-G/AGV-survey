# -*- coding: utf-8 -*-

""" avro python class for file: ObjectPose3D """

import json
from helpers import default_json_serialize, todict
from typing import Union
from AGVPoint3D import AGVPoint3D
from AGVMatrix3x3 import AGVMatrix3x3


class ObjectPose3D(object):

    schema = """
    {
        "type": "record",
        "name": "ObjectPose3D",
        "namespace": "agvsurvey.avro",
        "fields": [
            {
                "name": "position",
                "type": {
                    "type": "record",
                    "name": "AGVPoint3D",
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
                    "namespace": "agvsurvey.avro"
                }
            },
            {
                "name": "rotation",
                "type": {
                    "type": "record",
                    "name": "AGVMatrix3x3",
                    "fields": [
                        {
                            "name": "m00",
                            "type": "float"
                        },
                        {
                            "name": "m01",
                            "type": "float"
                        },
                        {
                            "name": "m02",
                            "type": "float"
                        },
                        {
                            "name": "m10",
                            "type": "float"
                        },
                        {
                            "name": "m11",
                            "type": "float"
                        },
                        {
                            "name": "m12",
                            "type": "float"
                        },
                        {
                            "name": "m20",
                            "type": "float"
                        },
                        {
                            "name": "m21",
                            "type": "float"
                        },
                        {
                            "name": "m22",
                            "type": "float"
                        }
                    ],
                    "namespace": "agvsurvey.avro"
                }
            },
            {
                "name": "RefFrameID",
                "type": "string"
            },
            {
                "name": "ObjectID",
                "type": "string"
            }
        ]
    }
    """

    def __init__(self, obj: Union[str, dict, 'ObjectPose3D']) -> None:
        if isinstance(obj, str):
            obj = json.loads(obj)

        elif isinstance(obj, type(self)):
            obj = obj.__dict__

        elif not isinstance(obj, dict):
            raise TypeError(
                f"{type(obj)} is not in ('str', 'dict', 'ObjectPose3D')"
            )

        self.set_position(obj.get('position', None))

        self.set_rotation(obj.get('rotation', None))

        self.set_RefFrameID(obj.get('RefFrameID', None))

        self.set_ObjectID(obj.get('ObjectID', None))

    def dict(self):
        return todict(self)

    def set_position(self, values: AGVPoint3D) -> None:

        self.position = AGVPoint3D(values)

    def get_position(self) -> AGVPoint3D:

        return self.position

    def set_rotation(self, values: AGVMatrix3x3) -> None:

        self.rotation = AGVMatrix3x3(values)

    def get_rotation(self) -> AGVMatrix3x3:

        return self.rotation

    def set_RefFrameID(self, value: str) -> None:

        if isinstance(value, str):
            self.RefFrameID = value
        else:
            raise TypeError("field 'RefFrameID' should be type str")

    def get_RefFrameID(self) -> str:

        return self.RefFrameID

    def set_ObjectID(self, value: str) -> None:

        if isinstance(value, str):
            self.ObjectID = value
        else:
            raise TypeError("field 'ObjectID' should be type str")

    def get_ObjectID(self) -> str:

        return self.ObjectID

    def serialize(self) -> None:
        return json.dumps(self, default=default_json_serialize)

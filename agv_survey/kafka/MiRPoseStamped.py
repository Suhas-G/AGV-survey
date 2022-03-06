# -*- coding: utf-8 -*-

""" avro python class for file: MiRPoseStamped """

import json
from .helpers import default_json_serialize, todict
from typing import Union
from .Pose3d import Pose3d
from .TimestampUnix import TimestampUnix


class MiRPoseStamped(object):

    schema = """
    {
        "type": "record",
        "name": "MiRPoseStamped",
        "namespace": "de.dfki.cos.mrk40.avro",
        "fields": [
            {
                "name": "refFrameId",
                "type": "string"
            },
            {
                "name": "pose",
                "type": {
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
            },
            {
                "name": "timestamp",
                "type": {
                    "type": "record",
                    "name": "TimestampUnix",
                    "fields": [
                        {
                            "name": "seconds",
                            "type": "long"
                        },
                        {
                            "name": "nseconds",
                            "type": "int",
                            "default": 0
                        }
                    ],
                    "namespace": "de.dfki.cos.mrk40.avro"
                }
            }
        ]
    }
    """

    def __init__(self, obj: Union[str, dict, 'MiRPoseStamped']) -> None:
        if isinstance(obj, str):
            obj = json.loads(obj)

        elif isinstance(obj, type(self)):
            obj = obj.__dict__

        elif not isinstance(obj, dict):
            raise TypeError(
                f"{type(obj)} is not in ('str', 'dict', 'MiRPoseStamped')"
            )

        self.set_refFrameId(obj.get('refFrameId', None))

        self.set_pose(obj.get('pose', None))

        self.set_timestamp(obj.get('timestamp', None))

    def dict(self):
        return todict(self)

    def set_refFrameId(self, value: str) -> None:

        if isinstance(value, str):
            self.refFrameId = value
        else:
            raise TypeError("field 'refFrameId' should be type str")

    def get_refFrameId(self) -> str:

        return self.refFrameId

    def set_pose(self, values: Pose3d) -> None:

        self.pose = Pose3d(values)

    def get_pose(self) -> Pose3d:

        return self.pose

    def set_timestamp(self, values: TimestampUnix) -> None:

        self.timestamp = TimestampUnix(values)

    def get_timestamp(self) -> TimestampUnix:

        return self.timestamp

    def serialize(self) -> None:
        return json.dumps(self, default=default_json_serialize)

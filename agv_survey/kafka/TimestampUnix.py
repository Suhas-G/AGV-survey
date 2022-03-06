# -*- coding: utf-8 -*-

""" avro python class for file: TimestampUnix """

import json
from .helpers import default_json_serialize, todict
from typing import Union


class TimestampUnix(object):

    schema = """
    {
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
    """

    def __init__(self, obj: Union[str, dict, 'TimestampUnix']) -> None:
        if isinstance(obj, str):
            obj = json.loads(obj)

        elif isinstance(obj, type(self)):
            obj = obj.__dict__

        elif not isinstance(obj, dict):
            raise TypeError(
                f"{type(obj)} is not in ('str', 'dict', 'TimestampUnix')"
            )

        self.set_seconds(obj.get('seconds', None))

        self.set_nseconds(obj.get('nseconds', 0))

    def dict(self):
        return todict(self)

    def set_seconds(self, value: int) -> None:

        if isinstance(value, int):
            self.seconds = value
        else:
            raise TypeError("field 'seconds' should be type int")

    def get_seconds(self) -> int:

        return self.seconds

    def set_nseconds(self, value: int) -> None:

        if isinstance(value, int):
            self.nseconds = value
        else:
            raise TypeError("field 'nseconds' should be type int")

    def get_nseconds(self) -> int:

        return self.nseconds

    def serialize(self) -> None:
        return json.dumps(self, default=default_json_serialize)

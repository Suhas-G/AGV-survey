# -*- coding: utf-8 -*-

""" avro python class for file: SimpleKey """

import json
from helpers import default_json_serialize, todict
from typing import Union


class SimpleKey(object):

    schema = """
    {
        "type": "record",
        "name": "SimpleKey",
        "namespace": "de.dfki.cos.mrk40.avro",
        "fields": [
            {
                "name": "key",
                "type": "string"
            }
        ]
    }
    """

    def __init__(self, obj: Union[str, dict, 'SimpleKey']) -> None:
        if isinstance(obj, str):
            obj = json.loads(obj)

        elif isinstance(obj, type(self)):
            obj = obj.__dict__

        elif not isinstance(obj, dict):
            raise TypeError(
                f"{type(obj)} is not in ('str', 'dict', 'SimpleKey')"
            )

        self.set_key(obj.get('key', None))

    def dict(self):
        return todict(self)

    def set_key(self, value: str) -> None:

        if isinstance(value, str):
            self.key = value
        else:
            raise TypeError("field 'key' should be type str")

    def get_key(self) -> str:

        return self.key

    def serialize(self) -> None:
        return json.dumps(self, default=default_json_serialize)

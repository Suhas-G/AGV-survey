# -*- coding: utf-8 -*-

""" avro python class for file: AGVMatrix3x3 """

import json
from .helpers import default_json_serialize, todict
from typing import Union


class AGVMatrix3x3(object):

    schema = """
    {
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
    """

    def __init__(self, obj: Union[str, dict, 'AGVMatrix3x3']) -> None:
        if isinstance(obj, str):
            obj = json.loads(obj)

        elif isinstance(obj, type(self)):
            obj = obj.__dict__

        elif not isinstance(obj, dict):
            raise TypeError(
                f"{type(obj)} is not in ('str', 'dict', 'AGVMatrix3x3')"
            )

        self.set_m00(obj.get('m00', None))

        self.set_m01(obj.get('m01', None))

        self.set_m02(obj.get('m02', None))

        self.set_m10(obj.get('m10', None))

        self.set_m11(obj.get('m11', None))

        self.set_m12(obj.get('m12', None))

        self.set_m20(obj.get('m20', None))

        self.set_m21(obj.get('m21', None))

        self.set_m22(obj.get('m22', None))

    def dict(self):
        return todict(self)

    def set_m00(self, value: float) -> None:

        if isinstance(value, float):
            self.m00 = value
        else:
            raise TypeError("field 'm00' should be type float")

    def get_m00(self) -> float:

        return self.m00

    def set_m01(self, value: float) -> None:

        if isinstance(value, float):
            self.m01 = value
        else:
            raise TypeError("field 'm01' should be type float")

    def get_m01(self) -> float:

        return self.m01

    def set_m02(self, value: float) -> None:

        if isinstance(value, float):
            self.m02 = value
        else:
            raise TypeError("field 'm02' should be type float")

    def get_m02(self) -> float:

        return self.m02

    def set_m10(self, value: float) -> None:

        if isinstance(value, float):
            self.m10 = value
        else:
            raise TypeError("field 'm10' should be type float")

    def get_m10(self) -> float:

        return self.m10

    def set_m11(self, value: float) -> None:

        if isinstance(value, float):
            self.m11 = value
        else:
            raise TypeError("field 'm11' should be type float")

    def get_m11(self) -> float:

        return self.m11

    def set_m12(self, value: float) -> None:

        if isinstance(value, float):
            self.m12 = value
        else:
            raise TypeError("field 'm12' should be type float")

    def get_m12(self) -> float:

        return self.m12

    def set_m20(self, value: float) -> None:

        if isinstance(value, float):
            self.m20 = value
        else:
            raise TypeError("field 'm20' should be type float")

    def get_m20(self) -> float:

        return self.m20

    def set_m21(self, value: float) -> None:

        if isinstance(value, float):
            self.m21 = value
        else:
            raise TypeError("field 'm21' should be type float")

    def get_m21(self) -> float:

        return self.m21

    def set_m22(self, value: float) -> None:

        if isinstance(value, float):
            self.m22 = value
        else:
            raise TypeError("field 'm22' should be type float")

    def get_m22(self) -> float:

        return self.m22

    def serialize(self) -> None:
        return json.dumps(self, default=default_json_serialize)

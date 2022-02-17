from mir_control.mir_api import MirRestApi
from pprint import pprint
import json

api = MirRestApi()

points = []

while True:
    try:
        input()
        _, data = api.get_robot_status()
        pprint(data['position'])
        points.append(data['position'])

    except KeyboardInterrupt:
        d = json.dumps(points)
        with open('f', 'w') as file:
            file.write(d)

        break


    
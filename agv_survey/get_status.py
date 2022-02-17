from mir_control.mir_api import MirRestApi
from pprint import pprint
import json

api = MirRestApi()
_, data = api.get_robot_status()
pprint(data)
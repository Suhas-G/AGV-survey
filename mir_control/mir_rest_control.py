import base64
from uuid import uuid4
import requests

from config import MIR_AUTHORIZATION_CODE, MIR_ROS_HOST, MIR_WEB_SESSION_USERNAME, MIR_WEB_SESSION_PASSWORD

mir_headers = {
    "accept": "application/json",
    "Accept-Language": "en_US",
    "content-type": "application/json",
    "Authorization": "Basic " + MIR_AUTHORIZATION_CODE
}
class MirRestApi:
    def __init__(self) -> None:
        self.base_url = 'http://' + MIR_ROS_HOST + "/api/v2.0.0/"
        self.session = requests.Session()
        self.session.headers = mir_headers
        self.mission_id = None

    def go_to_position(self, pos_x, pos_y, orientation):
        res = self.session.post(self.base_url + 'actions/move_to_position',
                            json={
                                'parameters': [
                                    {'id': 'x', 'value': pos_x},
                                    {'id': 'y', 'value': pos_y},
                                    {'id': 'orientation', 'value': orientation}
                                ]
                            }
        )
        print(res.json())

    def add_mission(self, mission_name, mission_description):
        guid = uuid4()
        res = self.session.post(self.base_url + 'missions',
                                json = {
                                    'group_id': "c444fe63-d3a1-11e8-8c41-b8aeed74d1d4",
                                    'guid': str(guid),
                                    'name': mission_name,
                                    'description': mission_description,
                                }
                            )
        print(res.json())
        return (str(guid) if res.ok else None)

    def get_missions(self):
        res = self.session.get(self.base_url + 'missions')
        return res.json() if res.ok else []

    def get_actions(self, mission_id):
        res = self.session.get(self.base_url + 'missions/' + mission_id + '/actions')
        return res.json() if res.ok else []

    def initialize(self):
        missions = self.get_missions()
        missions = list(filter(lambda mission: mission['name'] == 'Go to position', missions))
        if len(missions) > 0:
            mission_id = missions[0]['guid']
        else:
            mission_id = self.add_mission('Go to position', 'A mission to go to a single position')
            if mission_id is None:
                return False
        
        print('Mission ID', mission_id)
        self.mission_id = mission_id
        actions = list(filter(lambda action: action['action_type'] == 'move_to_position', self.get_actions(mission_id)))
        for action in actions:
            res = self.session.delete(self.base_url + 'missions/' + mission_id + '/actions/' + action['guid'])
            if not res.ok:
                raise Exception('Deleting action failed:' + str(res.json()))

        
    def add_goals(self, goals):
        for goal in goals:
            ok, result = self.add_move_to_position_action(self.mission_id, goal['position']['x'], goal['position']['y'], goal['orientation'])
            if not ok:
                raise Exception('Adding action failed:' + str(result))

        

    def add_move_to_position_action(self, mission_id, pos_x, pos_y, orientation):
        res = self.session.post(self.base_url + 'missions/' + mission_id + '/actions',
                            json={
                                'action_type': 'move_to_position',
                                'parameters': [
                                    {'id': 'x', 'value': pos_x},
                                    {'id': 'y', 'value': pos_y},
                                    {'id': 'orientation', 'value': orientation},
                                    {"id": "retries", "value" : 1},
                                    {"id": "distance_threshold", "value": 0.25}
                                ],
                                'priority': 1
                            }
            )
        
        return res.ok, res.json

    def start(self):        
        res = self.session.post(self.base_url + 'mission_queue', json={'mission_id': self.mission_id})
        return res.ok, res.json()



api = MirRestApi()
# print(api.go_to_position_mission(4.804, 7.636, 0))
api.initialize()
api.add_goals(
    [
        {'position': {'x': 10.426, 'y': 7.042}, 'orientation': -176.427},
        {'position': {'x': 4.804, 'y': 7.636}, 'orientation': -38.781}
    ]
)

api.start()

        

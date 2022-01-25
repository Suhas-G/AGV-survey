import base64
from uuid import uuid4
import requests

from .config import MIR_ROS_HOST, MIR_WEB_SESSION_USERNAME, MIR_WEB_SESSION_PASSWORD

mir_headers = {
    "accept": "application/json",
    "Accept-Language": "en_US",
    "content-type": "application/json",
    "Authorization": 'Basic ' + base64.b64encode(bytes(MIR_WEB_SESSION_USERNAME + ':' + MIR_WEB_SESSION_PASSWORD, 'utf-8')).decode('utf-8')
}


class MirRestApi:
    def __init__(self) -> None:
        self.base_url = 'http://' + MIR_ROS_HOST + "/api/v2.0.0/"
        self.session = requests.Session()
        self.session.headers.update(mir_headers)

    def go_to_position(self, pos_x, pos_y, orientation):
        res = self.session.post(self.base_url + 'actions/move-to-coordinates',
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
                                    'group_id': None,
                                    'guid': str(guid),
                                    'name': mission_name,
                                    'description': mission_description,
                                }
                            )
        return (guid if res.ok else None)

    def get_missions(self):
        res = self.session.get(self.base_url + 'missions')
        return res.json() if res.ok else []

    def go_to_position_mission(self, pos_x, pos_y, orientation):
        missions = self.get_missions()
        missions = filter(lambda mission: mission['name'] == 'Go to position', missions)
        if len(missions) > 0:
            mission_id = missions[0]['guid']
        else:
            mission_id = self.add_mission('Go to position', 'A mission to go to a single position')
            if mission_id is None:
                return False
        
        res = self.session.post(self.base_url + 'missions/' + mission_id + '/actions',
                            json={
                                'action_type': 'move-to-coordinates',
                                'parameters': [
                                    {'id': 'x', 'value': pos_x},
                                    {'id': 'y', 'value': pos_y},
                                    {'id': 'orientation', 'value': orientation}
                                ],
                                'priority': 1
                            }
            )

        if not res.ok:
            return False
        
        res = self.session.post(self.base_url + 'mission_queue', json={'mission_id': mission_id})
        return res.ok, res.json()
        

        

from enum import Enum
from uuid import uuid4

import requests

from .config import (MIR_ACTION_TYPE, MIR_AUTHORIZATION_CODE,
                    MIR_MISSION_GROUP_ID, MIR_MISSION_NAME, MIR_ROS_HOST)


class MirStatus(Enum):
    """
    Enum class for the status of the MIR.
    """
    UNKNOWN = 'unknown'
    FINISHED = 'finished'
    FAILED = 'failed'
    PENDING =  'pending'
    EXECUTING = 'executing'
    SUCCEDED = 'succeded'
    DONE = 'done'


class MirRestApi:
    def __init__(self) -> None:
        self.base_url = 'http://' + MIR_ROS_HOST + "/api/v2.0.0/"
        self.session = requests.Session()
        self.session.headers = {
                            "accept": "application/json",
                            "Accept-Language": "en_US",
                            "content-type": "application/json",
                            "Authorization": "Basic " + MIR_AUTHORIZATION_CODE
                        }
        self.mission_id = None
        self.mission_queue_id = None

    def go_to_position(self, pos_x, pos_y, orientation):
        res = self.session.post(self.base_url + 'actions/' + MIR_ACTION_TYPE,
                            json={
                                'parameters': [
                                    {'id': 'x', 'value': pos_x},
                                    {'id': 'y', 'value': pos_y},
                                    {'id': 'orientation', 'value': orientation}
                                ]
                            }
        )

    def add_mission(self, mission_name, mission_description):
        guid = uuid4()
        res = self.session.post(self.base_url + 'missions',
                                json = {
                                    'group_id': MIR_MISSION_GROUP_ID,
                                    'guid': str(guid),
                                    'name': mission_name,
                                    'description': mission_description,
                                }
                            )
        return (str(guid) if res.ok else None)

    def get_missions(self):
        res = self.session.get(self.base_url + 'missions')
        return res.json() if res.ok else []

    def get_actions(self, mission_id):
        res = self.session.get(self.base_url + 'missions/' + mission_id + '/actions')
        return res.json() if res.ok else []

    def initialize(self):
        missions = self.get_missions()
        missions = list(filter(lambda mission: mission['name'] == MIR_MISSION_NAME, missions))
        if len(missions) > 0:
            mission_id = missions[0]['guid']
        else:
            mission_id = self.add_mission(MIR_MISSION_NAME, 'A mission to go to a single position')
            if mission_id is None:
                return False
        
        print('Mission ID', mission_id)
        self.mission_id = mission_id
        actions = list(filter(lambda action: action['action_type'] == MIR_ACTION_TYPE, 
                        self.get_actions(mission_id)))
        for action in actions:
            res = self.session.delete(self.base_url + 'missions/' + mission_id + '/actions/' 
                                        + action['guid'])
            if not res.ok:
                raise Exception('Deleting action failed:' + str(res.json()))

    def add_goals(self, goals):
        for goal in goals:
            ok, result = self.add_move_to_position_action(self.mission_id, goal['position']['x'], 
                                        goal['position']['y'], goal['orientation'])
            if not ok:
                raise Exception('Adding action failed:' + str(result))

    def add_move_to_position_action(self, mission_id, pos_x, pos_y, orientation):
        res = self.session.post(self.base_url + 'missions/' + mission_id + '/actions',
                            json={
                                'action_type': MIR_ACTION_TYPE,
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
        res = self.session.post(self.base_url + 'mission_queue', 
                                json={'mission_id': self.mission_id})
        if res.ok:
            self.mission_queue_id = str(res.json()['id'])
        return res.ok, res.json()

    def cancel_mission(self):
        if self.mission_queue_id is None:
            return False
        res = self.session.delete(self.base_url + 'mission_queue/' + 
                                    self.mission_queue_id)
        if res.ok:
            self.mission_queue_id = None
        return res.ok

    def get_mission_actions(self):
        if self.mission_queue_id is None:
            return []
        res = self.session.get(self.base_url + 'mission_queue/' + 
                                self.mission_queue_id + '/actions')
        return res.json() if res.ok else []

    def get_mission_status(self):
        if self.mission_queue_id is None:
            return MirStatus.UNKNOWN
        res = self.session.get(self.base_url + 'mission_queue/' + self.mission_queue_id)
        return MirStatus(res.json()['state'].lower()) if res.ok else MirStatus.UNKNOWN

    def get_action_details(self, action_id):
        if self.mission_queue_id is None:
            return None
        res = self.session.get(self.base_url + 'mission_queue/' + self.mission_queue_id + 
                                '/actions/' + action_id)
        return res.json() if res.ok else None

    def get_robot_status(self):
        res = self.session.get(self.base_url + 'status')
        return res.ok, res.json()

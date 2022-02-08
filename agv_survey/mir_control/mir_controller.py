import math
import threading
from time import time
from typing import List
import numpy as np

from mir_api import MirRestApi, MirStatus


class MirController:
    def __init__(self, goals: List[dict], interval = 1) -> None:
        self.api = MirRestApi()
        self.goals = [dict(goal, **{'status': None}) for goal in goals]
        self.interval = interval
        self.is_running = False
        self.data = {'pose': None, 'state': None, 'current_goal': None}
        self.data_lock = threading.Lock()
        self.next_call = time()
        self.start()


    def monitor(self):
        if self.api.get_mission_status() == MirStatus.SUCCEEDED:
            self.stop()
        
        successful, result = self.api.get_robot_status()
        if not successful:
            return

        self.data_lock.acquire()
        self.data['pose'] = result['position']
        self.data['state'] = result['state_id']
        self.data_lock.release()


        actions = self.api.get_mission_actions()
        for i, action in enumerate(actions):
            action_state = MirStatus(action['state'].lower())
            action_data = {}
            for parameter in action['parameters']:
                if parameter['id'] in ('x', 'y', 'orientation'):
                    action_data[parameter['id']] = parameter['value']

            assert (math.isclose(self.goals[i]['position']['x'], action_data['x'], rel_tol=0.05))
            assert (math.isclose(self.goals[i]['position']['y'], action_data['y'], rel_tol=0.05))
            assert (math.isclose(self.goals[i]['orientation'], action_data['orientation'], rel_tol=0.05))

            if action_state == MirStatus.SUCCEEDED:
                self.goals[i]['status'] = MirStatus.SUCCEEDED

            if action_state == MirStatus.PENDING and action['action_type'] == 'move_to_position':
                self.data_lock.acquire()
                self.data['current_goal'] = {}
                self.data['current_goal']['position'] = {'x': action_data['x'], 'y': action_data['y']}
                self.data['current_goal']['orientation'] = action_data['orientation']
                self.data['current_goal']['goal_index'] = i
                self.data_lock.release()

    def start(self):
        if not self.is_running:
            self.api.initialize()
            self.api.add_goals(self.goals)
            self.api.start()
            self.next_call += self.interval
            # self._timer = threading.Timer(self.next_call - time(), self.monitor)
            self._timer = threading.Timer(interval=self.interval, function=self.monitor)
            self._timer.start()
            self.is_running = True

    def cancel(self):
        successful = self.api.cancel_mission()
        if successful:
            self.stop()
        
        return successful

    def stop(self):
        if self.is_running:
            self._timer.cancel()
            self.is_running = False

    def get_pose(self):
        pose = None
        self.data_lock.acquire()
        if self.is_running and self.data['pose'] is not None:
            orientation = self.data['pose']['orientation']
            x = self.data['pose']['x']
            y = self.data['pose']['y']
            pose = np.array([
                [np.cos(orientation), -np.sin(orientation), 0, x],
                [np.sin(orientation), np.cos(orientation), 0, y],
                [0, 0, 1, 0],
                [0, 0, 0, 1]
            ])
        self.data_lock.release()
        return pose

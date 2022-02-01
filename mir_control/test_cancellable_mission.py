import threading
from time import sleep, time
from typing import List
from mir_rest_control import MirRestApi, MirStatus
import math


class MirController:
    def __init__(self, goals: List[dict], interval = 1) -> None:
        self.api = MirRestApi()
        self.goals = [dict(goal, **{'status': None}) for goal in goals]
        self.interval = interval
        self.is_running = False
        self.data = {'pose': None, 'state': None, 'current_goal': None}
        self.next_call = time()
        self.start()


    def monitor(self):
        if self.api.get_mission_status() == MirStatus.COMPLETED:
            self.stop()
        
        successful, result = self.api.get_robot_status()
        if successful:
            self.data['pose'] = result['position']
            self.data['state'] = result['state_id']

        actions = self.api.get_mission_actions()
        for i, action in enumerate(actions):
            action_state = MirStatus(action['state'])
            action_data = {}
            for parameter in action['parameters']:
                if parameter['id'] in ('x', 'y', 'orientation'):
                    action_data[parameter['id']] = parameter['value']

            assert (math.isclose(self.goals[i]['position']['x'], action_data['x'], rel_tol=0.05))
            assert (math.isclose(self.goals[i]['position']['y'], action_data['y'], rel_tol=0.05))
            assert (math.isclose(self.goals[i]['orientation'], action_data['orientation'], rel_tol=0.05))

            if action_state == MirStatus.COMPLETED:
                self.goals[i]['status'] = MirStatus.COMPLETED

            if action_state == MirStatus.EXECUTING and action['action_type'] == 'move_to_position':
                self.data['current_goal'] = {}
                self.data['current_goal']['position'] = {'x': action_data['x'], 'y': action_data['y']}
                self.data['current_goal']['orientation'] = action_data['orientation']
                self.data['current_goal']['goal_index'] = i

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



def main():
    print('Starting MiR mission...')
    controller = MirController([
        {'position': {'x': 10.426, 'y': 7.042}, 'orientation': -176.427},
        {'position': {'x': 4.804, 'y': 7.636}, 'orientation': -38.781}
    ])
    try:
        while True:
            sleep(2)
            print('Current Robot pose:', controller.data['pose'])
            print('Current Goal:', controller.data['current_goal'])
    except KeyboardInterrupt:
        print('Cancelling mission...')
        controller.cancel()
        print('Mission cancelled.')
    finally:
        controller.stop()


if __name__ == '__main__':
    main()
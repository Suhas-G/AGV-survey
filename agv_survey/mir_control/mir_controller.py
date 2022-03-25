import math
import threading
from time import time
from typing import List
import numpy as np

from .mir_api import MirRestApi, MirStatus
from copy import deepcopy


class MirController:
    def __init__(self, interval = 1) -> None:
        self.api = MirRestApi()
        self.interval = interval
        self.is_running = False
        self.data = {'pose': None, 'state': None, 'current_goal': None}
        self.done = True
        self.data_lock = threading.Lock()
        self.next_call = time()
        self.start()


    def monitor_mission_status(self):
        print('Checking mission status')
        if self.is_running:
            mission_status = self.api.get_mission_status()
            print('Mission status', mission_status)
            if mission_status == MirStatus.DONE:
                print('Mission done')
                self.data_lock.acquire()
                self.done = True
                self.data_lock.release()
                self.stop()

        if self.is_running:
            self._timer = threading.Timer(interval=self.interval, function=self.monitor_mission_status)
            self._timer.start()

    def is_done(self):
        self.data_lock.acquire()
        done = self.done
        self.data_lock.release()
        return done

    def start_mission(self):
        self.is_running = True
        self.api.start_mission()
        self.next_call += self.interval
            # self._timer = threading.Timer(self.next_call - time(), self.monitor)
        self._timer = threading.Timer(interval=self.interval, function=self.monitor_mission_status)
        self._timer.start()


    def rotate(self, angle_degrees):
        self.done = False
        self.api.move_relative(0, 0, angle_degrees)
        self.start_mission()


    def move_to_position(self, goal):
        self.done = False
        self.api.move_to_position(goal['position']['x'], goal['position']['y'], goal['orientation'])
        self.start_mission()


    def start(self):
        if not self.is_running:
            self.api.initialize()
            # self.api.add_goals(self.goals)
            # self.api.start()
            # self.next_call += self.interval
            # # self._timer = threading.Timer(self.next_call - time(), self.monitor)
            # self._timer = threading.Timer(interval=self.interval, function=self.monitor_mission_status)
            # self._timer.start()
            self.is_running = True

    def cancel(self):
        print('Cancelling')
        successful = self.api.cancel_mission()
        if successful:
            self.stop()
        
        return successful

    def stop(self):
        print('Stopping')
        if self.is_running:
            self._timer.cancel()
            self.is_running = False

    def get_pose(self):
        pose = None
        self.data_lock.acquire()
        if self.is_running and self.data['pose'] is not None:
            orientation = self.data['pose']['orientation'] * np.pi / 180
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

    def get_data(self):
        self.data_lock.acquire()
        data = deepcopy(self.data)
        self.data_lock.release()
        return data

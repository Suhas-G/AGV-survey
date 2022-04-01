import threading
from time import time

import numpy as np

from .mir_api import MirRestApi, MirStatus


class MirController:
    def __init__(self, interval = 1) -> None:
        self.api = MirRestApi()
        self.interval = interval
        self.is_running = False
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
        self._timer = threading.Timer(interval=self.interval, function=self.monitor_mission_status)
        self._timer.start()


    def rotate(self, angle_degrees):
        self.data_lock.acquire()
        self.done = False
        self.data_lock.release()
        self.api.move_relative(0, 0, angle_degrees)
        self.start_mission()


    def move_to_position(self, goal):
        self.data_lock.acquire()
        self.done = False
        self.data_lock.release()
        self.api.move_to_position(goal['position']['x'], goal['position']['y'], goal['orientation'])
        self.start_mission()


    def start(self):
        if not self.is_running:
            self.api.initialize()
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


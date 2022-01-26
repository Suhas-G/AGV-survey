from time import sleep, time
from enum import Enum
import uuid
import roslibpy
import roslibpy.actionlib

import mirrosbridge as mb

from config import MIR_ROS_HOST, MIR_ROS_PORT

from collections import deque


class GoalStatus(Enum):
    PENDING = 0   # The goal has yet to be processed by the action server
    ACTIVE = 1   # The goal is currently being processed by the action server
    PREEMPTED = 2   # The goal received a cancel request after it started executing
                                #   and has since completed its execution (Terminal State)
    # The goal was achieved successfully by the action server (Terminal State)
    SUCCEEDED = 3
    ABORTED = 4   # The goal was aborted during execution by the action server due
                                #    to some failure (Terminal State)
    REJECTED = 5   # The goal was rejected by the action server without being processed,
                                #    because the goal was unattainable or invalid (Terminal State)
    PREEMPTING = 6   # The goal received a cancel request after it started executing
                                #    and has not yet completed execution
    RECALLING = 7   # The goal received a cancel request before it started executing,
                                #    but the action server has not yet confirmed that the goal is canceled
    RECALLED = 8   # The goal received a cancel request before it started executing
                                #    and was successfully cancelled (Terminal State)
    LOST = 9   # An action client can determine that a goal is LOST. This should not be
                                #    sent over the wire by an action server


class MirPathPlanner:
    def __init__(self, goals) -> None:
        self.connection = mb.MirRosBridge(MIR_ROS_HOST, MIR_ROS_PORT)
        self.pose_action = self.connection.get_action_client(
            'move_base', 'move_base_msgs/MoveBaseAction')

        self.goals = deque(goals)

    def start(self):
        self.publish_goal()

    def publish_goal(self):
        goal_data = self.goals.pop()
        now = roslibpy.Time.now()
        message = roslibpy.Message({
            'header': {
                'stamp': {'secs': now.secs, 'nsecs': now.nsecs},
                'frame_id': 'map'
            },
            'goal_id': {
                'stamp': {'secs': now.secs, 'nsecs': now.nsecs},
                'id': str(uuid.uuid4())
            },
            'goal': {
                'target_pose': {
                    'header': {
                        'stamp': {'secs': now.secs, 'nsecs': now.nsecs},
                    },
                    'pose': {
                        'position': {'x': goal_data['position']['x'], 'y': goal_data['position']['y'], 'z': 0.0},
                        'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}
                    }
                },
            }
        })
        self.goal = roslibpy.actionlib.Goal(self.pose_action, message)
        self.goal.send(self.goal_status_update)
        print('Published goal:', goal_data['position']['x'], goal_data['position']['y'])

    def goal_status_update(self, data):
        if isinstance(data, dict):
            if data.get('status', {}).get('status', -1) == GoalStatus.SUCCEEDED.value:
                print('Goal reached:', data)

                if len(self.goals) > 0:
                    self.publish_goal()
                else:
                    self.pose_action.cancel()
                    self.pose_action.dispose()
                    self.connection.close_bridge()
            else:
                print('Goal not reached:', data)
        else:
            print('Unknown data type!')


def main():
    planner = MirPathPlanner(goals=[
        {'position': {'y': 7.636, 'x': 4.804, 'z': 0.0}, 'orientation': {
            'y': 0.0, 'x': 0.0, 'z': 0.999280633973721, 'w': -0.03792379945467261}},
        # {'position': {'y': 6.414441751347013, 'x': 9.829257192968441, 'z': 0.0}, 'orientation': {
        #     'y': 0.0, 'x': 0.0, 'z': 0.9738176415775766, 'w': 0.22733059836345568}},

        # {'position': {'y': 6.9090162980103695, 'x': 7.656512662866348, 'z': 0.0}, 'orientation': {
        #     'y': 0.0, 'x': 0.0, 'z': 0.9993338758450409, 'w': -0.036493897962927686}},

        # {'position': {'y': 7.510136324982207, 'x': 5.6111179316265964, 'z': 0.0}, 'orientation': {
        #     'y': 0.0, 'x': 0.0, 'z': 0.03910976522601516, 'w': 0.9992349204586307}},
        # {'position': {'y': 8.19341996689738, 'x': 8.611763794704805, 'z': 0.0}, 'orientation': {
        #     'y': 0.0, 'x': 0.0, 'z': 0.299858083373351, 'w': 0.9539838205313864}},
        # {'position': {'y': 7.220134532134866, 'x': 10.273797408916966, 'z': 0.0}, 'orientation': {
        #     'y': 0.0, 'x': 0.0, 'z': 0.03343908674387113, 'w': 0.9994407573627042}}
        ])
    planner.start()

    while True:
        sleep(1)
        print(planner.goal.feedback)

def list_topics():
    connection=mb.MirRosBridge(MIR_ROS_HOST, MIR_ROS_PORT)
    print(connection.get_topics())

def list_services():
    connection=mb.MirRosBridge(MIR_ROS_HOST, MIR_ROS_PORT)
    print(connection.get_services())


def observe_pose():
    connection=mb.MirRosBridge(MIR_ROS_HOST, MIR_ROS_PORT)

    while True:
        sleep(1)
        print(connection.data['pose'])

if __name__ == '__main__':
    # main()
    # list_topics()
    observe_pose()

from time import time
from enum import Enum
import uuid
import roslibpy

import mirrosbridge as mb

from .config import MIR_ROS_HOST, MIR_ROS_PORT

from collections import deque


class GoalStatus(Enum):
    PENDING         = 0   # The goal has yet to be processed by the action server
    ACTIVE          = 1   # The goal is currently being processed by the action server
    PREEMPTED       = 2   # The goal received a cancel request after it started executing
                                #   and has since completed its execution (Terminal State)
    SUCCEEDED       = 3   # The goal was achieved successfully by the action server (Terminal State)
    ABORTED         = 4   # The goal was aborted during execution by the action server due
                                #    to some failure (Terminal State)
    REJECTED        = 5   # The goal was rejected by the action server without being processed,
                                #    because the goal was unattainable or invalid (Terminal State)
    PREEMPTING      = 6   # The goal received a cancel request after it started executing
                                #    and has not yet completed execution
    RECALLING       = 7   # The goal received a cancel request before it started executing,
                                #    but the action server has not yet confirmed that the goal is canceled
    RECALLED        = 8   # The goal received a cancel request before it started executing
                                #    and was successfully cancelled (Terminal State)
    LOST            = 9   # An action client can determine that a goal is LOST. This should not be
                                #    sent over the wire by an action server

class MirPathPlanner:
    def __init__(self, goals) -> None:
        self.connection = mb.MirRosBridge(MIR_ROS_HOST, MIR_ROS_PORT)
        self.pose_action = self.connection.get_action_client('move_base', 'move_base_msgs/MoveBaseAction')

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
                        'position': {'x': goal_data[0], 'y': goal_data[1], 'z': 0.0},
                        'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}
                    }
                },
            }
        })
        self.goal = roslibpy.actionlib.Goal(self.pose_action, message)
        self.goal.send(self.goal_status_update)
        print('Published goal:', goal_data[0], goal_data[1]) 

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
    planner = MirPathPlanner(goals=[(10.0, 2.0), (18.0, 4.0), (17.0, 9.0), (8.0, 9.0)])
    planner.start()

def list_topics():
    connection = mb.MirRosBridge(MIR_ROS_HOST, MIR_ROS_PORT)
    print(connection.get_topics())

def list_services():
    connection = mb.MirRosBridge(MIR_ROS_HOST, MIR_ROS_PORT)
    print(connection.get_services())

if __name__ == '__main__':
    # main()
    list_topics()
from time import time
import roslibpy

import mirrosbridge as mb

from .config import MIR_ROS_HOST, MIR_ROS_PORT, MOVE_BASE_GOAL_TOPIC, MOVE_BASE_RESULT_TOPIC

from collections import deque

class MirPathPlanner:
    def __init__(self, goals) -> None:
        self.connection = mb.MirRosBridge(MIR_ROS_HOST, MIR_ROS_PORT)
        self.pose_publisher = self.connection.get_publisher_to_topic(MOVE_BASE_GOAL_TOPIC, 
                                                                    'geometry_msgs/PoseStamped')

        self.goals = deque(goals)

    def start(self):
        self.connection.subscribe_to_topic(MOVE_BASE_RESULT_TOPIC, 'move_base_msgs/MoveBaseActionResult', 
                                            self.goal_status_update)
        self.publish_goal()

    def publish_goal(self):
        goal = self.goals.pop()
        message = roslibpy.Message({
            'header': {
                'stamp': int(time() * 1000),
                'frame_id': 'map'
            },
            'pose': {
                'position': {'x': goal[0], 'y': goal[1]}
            }
        })

        self.pose_publisher.publish(message)
        print('Published goal:', goal[0], goal[1]) 

    def goal_status_update(self, data):
        if isinstance(data, dict):
            if data.get('status', {}).get('status', -1) == 3:
                print('Goal reached:', data)

                if len(self.goals) > 0:
                    self.publish_goal()
                else:
                    self.pose_publisher.unadvertise()
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
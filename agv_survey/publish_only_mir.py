import time

import numpy as np

from mir_control.mir_api import MirRestApi
from kafka.kafka_producer import get_producer, send_pose3d, send_mir_pose
from kafka.AGVPoint3D import AGVPoint3D
from kafka.ObjectPose3D import ObjectPose3D
from kafka.AGVMatrix3x3 import AGVMatrix3x3 
from kafka.MiRPoseStamped import MiRPoseStamped
from kafka.Pose3d import Pose3d
from kafka.TimestampUnix import TimestampUnix


def get_robot_position(api):
    _, data = api.get_robot_status()
    robot_position = data['position']
    print('Robot Location:', data['position'])
    return robot_position

def to_quaternio(yaw, pitch, roll):

    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)


    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return {'x': x, 'y': y, 'z': z, 'w': w}

def get_mir_pose(x, y, z, orientation):
    return Pose3d({'position': {'x': x, 'y': y, 'z': z}, 'orientation': orientation})

def get_timestamp():
    nseconds = int(time.time_ns())
    seconds = int(time.time())
    return TimestampUnix({'seconds': seconds, 'nseconds': nseconds})

def get_mir_pose_stamped(pose, timestamp):
    return MiRPoseStamped({'pose': pose, 'timestamp': timestamp, 'refFrameId': '10'})

def publish_mir_pose(x, y, orientation_degrees, producer_mir):
    orientation_radians = np.radians(orientation_degrees)
    mir_pose = get_mir_pose(x, y, 0.0, to_quaternio(orientation_radians, 0, 0))
    timestamp = get_timestamp()
    mir_pose_stamped = get_mir_pose_stamped(mir_pose, timestamp)
    send_mir_pose(mir_pose_stamped, producer_mir)


def main():
    api = MirRestApi()
    _, producer_mir = get_producer()
    try:
        while True:
            robot_position = get_robot_position(api)
            publish_mir_pose(robot_position['x'], robot_position['y'], robot_position['orientation'], producer_mir)
            pass
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()
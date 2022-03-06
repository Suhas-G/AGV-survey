import numpy as np

from camera import RealSenseCamera
from detection import CameraIntrinsic, MediaPipeDetection
from mir_control.mir_api import MirRestApi
from object_tracking_store import ObjectTrackingStore

from kafka.kafka_producer import get_producer, send_pose3d, send_mir_pose
from kafka.AGVPoint3D import AGVPoint3D
from kafka.ObjectPose3D import ObjectPose3D
from kafka.AGVMatrix3x3 import AGVMatrix3x3 
from kafka.MiRPoseStamped import MiRPoseStamped
from kafka.Pose3d import Pose3d
from kafka.TimestampUnix import TimestampUnix
import math, time
import cv2
PUBLISH = True

DEBUG = True

def to_quaternio(yaw, pitch, roll):

    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)


    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return {'x': x, 'y': y, 'z': z, 'w': w}


def draw_object_center(image, center, rect):
    cv2.circle(image, center, 3, [255, 255, 0], -1)
    cv2.rectangle(image, (rect[0], rect[1]), (rect[0] + rect[2], rect[1] + rect[3]), [255, 0, 255], 2)

def draw_object_position(image, center, x, y, depth):
    cv2.putText(image, str(depth), (center[0] + 20, center[1] + 20), cv2.FONT_HERSHEY_SCRIPT_COMPLEX, 0.5, (255,255,255))
    cv2.putText(image, str(x), (center[0] + 20, center[1] + 40), cv2.FONT_HERSHEY_SCRIPT_COMPLEX, 0.5, (255,255,255))
    cv2.putText(image, str(y), (center[0] + 20, center[1] + 60), cv2.FONT_HERSHEY_SCRIPT_COMPLEX, 0.5, (255,255,255))




def get_point3d(x, y, z) -> AGVPoint3D:
    return AGVPoint3D({'x': float(x), 'y': float(y), 'z': float(z)})

def get_rotation(rotation_matrix) -> AGVMatrix3x3:
    data = {}
    for i in range(len(rotation_matrix)):
        for j in range(len(rotation_matrix[i])):
            data['m{}{}'.format(i, j)] = float(rotation_matrix[i, j])
    return AGVMatrix3x3(data)

def get_object_pose(position, rotation, object_id) -> ObjectPose3D:
    return ObjectPose3D({'position': position, 'rotation': rotation, 'RefFrameID': '10', 'ObjectID': str(object_id)})


def get_mir_pose(x, y, z, orientation):
    return Pose3d({'position': {'x': x, 'y': y, 'z': z}, 'orientation': orientation})

def get_timestamp():
    nseconds = int(time.time_ns())
    seconds = int(time.time())
    return TimestampUnix({'seconds': seconds, 'nseconds': nseconds})

def get_mir_pose_stamped(pose, timestamp):
    return MiRPoseStamped({'pose': pose, 'timestamp': timestamp, 'refFrameId': '10'})


def get_bounding_box_xywh(center, delta):
    return (center[0] - delta, center[1] - delta, 2 * delta, 2 * delta)

def get_pose(position):
    pose = None
    orientation = position['orientation']
    x = position['x']
    y = position['y']
    pose = np.array([
        [np.cos(orientation), -np.sin(orientation), 0, x],
        [np.sin(orientation), np.cos(orientation), 0, y],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])
    return pose

def main():
    api = MirRestApi()
    store = ObjectTrackingStore()
    camera = RealSenseCamera(simulate=False)
    intrinsic = camera.intrinsic
    detector = MediaPipeDetection(CameraIntrinsic(intrinsic.width, intrinsic.height, 
                                    intrinsic.fx, intrinsic.fy, intrinsic.ppx, intrinsic.ppy))
    print(detector.camera_intrinsic)
    if PUBLISH:
        producer_pose, producer_mir = get_producer()
    camera_pose = np.array([
        [1, 0, 0, 0.34],
        [0, 1, 0, 0],
        [0, 0, 1, 0.79],
        [0, 0, 0, 1]
    ])

    while True:
        try:
            input()
            image, depth = camera.get_rgb_depth(depth_for_display=False)
            results = detector.process(image, depth)
            if DEBUG:
                output = detector.draw_boxes(results, image.copy())
            coordinates, rotations = detector.get_pixel_coordinates(results)
            _, data = api.get_robot_status()
            robot_pose = get_pose(data['position'])
            for object_coordinates, rotation in zip(coordinates, rotations):
                coordinate = object_coordinates[0]
                center = tuple(map(int, coordinate))
                if (center[0] > image.shape[0] or center[1] > image.shape[1]):
                        continue
                rect = get_bounding_box_xywh(center, 10)
                x, y, depth_val = camera.get_3d_coordinates(rect, center)
                object_position = robot_pose @ (camera_pose @ np.array([depth_val, x, -y, 1]))
                object_id = store.query_and_push(object_position[:-1])

                # print('Object ID: {}. \nPosition: {}\nOrientation : {}'.format(object_id, object_position, rotation))

                if PUBLISH:
                    position = get_point3d(object_position[0], object_position[1], object_position[2])
                    rotation = get_rotation(rotation)
                    object_pose = get_object_pose(position, rotation, object_id)
                    print(object_pose.dict())
                    # send_pose3d(object_pose, producer_pose)

            if PUBLISH:
                mir_pose = get_mir_pose(data['position']['x'], data['position']['y'], 0.0, to_quaternio(data['position']['orientation'], 0, 0))
                timestamp = get_timestamp()
                mir_pose_stamped = get_mir_pose_stamped(mir_pose, timestamp)
                print(mir_pose_stamped.dict())
                send_mir_pose(mir_pose_stamped, producer_mir)

            if DEBUG:
                cv2.imshow('MediaPipe Objectron', cv2.cvtColor(output, cv2.COLOR_RGB2BGR))
                cv2.waitKey(10)

        except KeyboardInterrupt:
            break

if __name__ == '__main__':
    main()

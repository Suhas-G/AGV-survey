import cv2
from detection import CameraIntrinsic, DopeDetection
from camera import RealSenseCamera
import numpy as np

from pathlib import Path
from matplotlib import pyplot as plt

from mir_control.mir_api import MirRestApi
from object_tracking_store import ObjectTrackingStore
from kafka.kafka_producer import get_producer, send_pose3d, send_mir_pose
from kafka.AGVPoint3D import AGVPoint3D
from kafka.ObjectPose3D import ObjectPose3D
from kafka.AGVMatrix3x3 import AGVMatrix3x3 
from kafka.MiRPoseStamped import MiRPoseStamped
from kafka.Pose3d import Pose3d
from kafka.TimestampUnix import TimestampUnix
import time

DEBUG = True
PUBLISH = False

def get_pose(position):
    pose = None
    orientation = position['orientation'] *np.pi/180
    x = position['x']
    y = position['y']
    pose = np.array([
        [np.cos(orientation), -np.sin(orientation), 0, x],
        [np.sin(orientation), np.cos(orientation), 0, y],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])
    return pose

def get_bounding_box_xywh(center, delta):
    return (center[0] - delta, center[1] - delta, 2 * delta, 2 * delta)

def draw_object_center(image, center, rect):
    cv2.circle(image, center, 3, [255, 255, 0], -1)
    cv2.rectangle(image, (rect[0], rect[1]), (rect[0] + rect[2], rect[1] + rect[3]), [255, 0, 255], 2)

def draw_object_position(image, center, x, y, depth):
    cv2.putText(image, str(depth), (center[0] + 20, center[1] + 20), cv2.FONT_HERSHEY_SCRIPT_COMPLEX, 0.5, (255,255,255))
    cv2.putText(image, str(x), (center[0] + 20, center[1] + 40), cv2.FONT_HERSHEY_SCRIPT_COMPLEX, 0.5, (255,255,255))
    cv2.putText(image, str(y), (center[0] + 20, center[1] + 60), cv2.FONT_HERSHEY_SCRIPT_COMPLEX, 0.5, (255,255,255))

def draw_object_coords(image, center, x, y, z, object_id):
    cv2.putText(image, 'X: ' + str(round(x, 3)), (center[0] + 20, center[1] + 20), cv2.FONT_HERSHEY_SCRIPT_COMPLEX, 0.5, (255,255,255))
    cv2.putText(image, 'Y: ' + str(round(y, 3)), (center[0] + 20, center[1] + 40), cv2.FONT_HERSHEY_SCRIPT_COMPLEX, 0.5, (255,255,255))
    cv2.putText(image, 'Z: ' + str(round(z, 3)), (center[0] + 20, center[1] + 60), cv2.FONT_HERSHEY_SCRIPT_COMPLEX, 0.5, (255,255,255))
    cv2.putText(image, 'ID: ' + str(object_id), (center[0] + 20, center[1] + 80), cv2.FONT_HERSHEY_SCRIPT_COMPLEX, 0.5, (255,255,255))

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

def quaternion_rotation_matrix(Q):
    """
    Covert a quaternion into a full three-dimensional rotation matrix.
 
    Input
    :param Q: A 4 element array representing the quaternion (q0,q1,q2,q3) 
 
    Output
    :return: A 3x3 element matrix representing the full 3D rotation matrix. 
             This rotation matrix converts a point in the local reference 
             frame to a point in the global reference frame.
    """
    # Extract the values from Q
    q0 = Q[0]
    q1 = Q[1]
    q2 = Q[2]
    q3 = Q[3]
     
    # First row of the rotation matrix
    r00 = 2 * (q0 * q0 + q1 * q1) - 1
    r01 = 2 * (q1 * q2 - q0 * q3)
    r02 = 2 * (q1 * q3 + q0 * q2)
     
    # Second row of the rotation matrix
    r10 = 2 * (q1 * q2 + q0 * q3)
    r11 = 2 * (q0 * q0 + q2 * q2) - 1
    r12 = 2 * (q2 * q3 - q0 * q1)
     
    # Third row of the rotation matrix
    r20 = 2 * (q1 * q3 - q0 * q2)
    r21 = 2 * (q2 * q3 + q0 * q1)
    r22 = 2 * (q0 * q0 + q3 * q3) - 1
     
    # 3x3 rotation matrix
    rot_matrix = np.array([[r00, r01, r02],
                           [r10, r11, r12],
                           [r20, r21, r22]])
                            
    return rot_matrix

def publish_object_pose(object_position, rotation, object_id, producer_object_pose):
    rotation_matrix = quaternion_rotation_matrix(rotation)
    position = get_point3d(object_position[0], object_position[1], object_position[2])
    rotation = get_rotation(rotation_matrix)
    object_pose = get_object_pose(position, rotation, object_id)
    send_pose3d(object_pose, producer_object_pose)



def main():
    api = MirRestApi()
    store = ObjectTrackingStore()
    camera = RealSenseCamera(simulate=False, filepath='./data/bluebox.bag')
    intrinsic = camera.intrinsic
    detector = DopeDetection(intrinsic, './config.yaml')

    camera_pose = np.array([
        [1, 0, 0, 0.34],
        [0, 1, 0, 0],
        [0, 0, 1, 0.79],
        [0, 0, 0, 1]
    ])

    if PUBLISH:
        producer_object_pose, producer_mir = get_producer()

    try:
        while True:
            # input()
            # sleep(5)
            _, data = api.get_robot_status()
            robot_position = data['position']
            robot_pose = get_pose(data['position'])
            print('Robot Location:', data['position'])

            if PUBLISH:
                publish_mir_pose(robot_position['x'], robot_position['y'], robot_position['orientation'], producer_mir)

            image, depth = camera.get_rgb_depth(depth_for_display=False)
            # image = cv2.resize(cv2.cvtColor(image, cv2.COLOR_BGR2RGB), (640, 480))
            results = detector.process(image, depth)
            # print('Detected Objects:', len(results))
            coordinates, rotations = detector.get_pixel_coordinates(results)
            if DEBUG:
                output = detector.draw_boxes(results, image.copy())
            for object_coordinates, rotation in zip(coordinates, rotations):
                center = tuple(map(int, list(object_coordinates[-1])))

                if (center[0] < 0 or center[1] < 0 or center[0] > intrinsic.width or center[1] > intrinsic.height):
                    continue
                rect = get_bounding_box_xywh(center, 10)
                x, y, depth_val = camera.get_3d_coordinates(rect, center)

                object_position = robot_pose @ (camera_pose @ np.array([depth_val, -x, -y, 1]))
                object_id, belief_count = store.query_and_push(object_position[:-1])
                print('Object: ', object_id, 'Position:', object_position)

                if DEBUG:
                    draw_object_center(output, center, rect)
                    # draw_object_position(output, center, x, y, depth_val)
                    draw_object_coords(output, center, object_position[0], object_position[1], object_position[2], object_id)

                if PUBLISH and belief_count > 5:
                    publish_object_pose(object_position[:-1], rotation, object_id, producer_object_pose)

            

            cv2.imshow('Dope Detection', output)
            cv2.waitKey(10)
            # if len(results) > 0:
            #     cv2.waitKey(0)
            # else:
            #     cv2.waitKey(10)
            # plt.close()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
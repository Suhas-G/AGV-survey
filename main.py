import time
import numpy as np
import cv2

from object_tracking_store import ObjectTrackingStore
from camera import RealSenseCamera
from detection.dope_detection import DopeDetection
from mir_control.mir_controller import MirController
from kafka.producer import get_producer, send_pose3d, send_mir_pose
from kafka.AGVPoint3d import AGVPoint3D
from kafka.ObjectPose3d import ObjectPose3D
from kafka.AGVMatrix3x3 import AGVMatrix3x3 
from kafka.MiRPoseStamped import MiRPoseStamped
from kafka.Pose3d import Pose3d
from kafka.TimestampUnix import TimestampUnix
from utils import (quaternion_to_euler, to_quaternion, get_bounding_box_xywh, 
                    draw_object_coords, quaternion_rotation_matrix)
from waypoint_manager import WaypointManager

DEBUG = True
PUBLISH = True
TIME = False


WAYPOINTS = [{'orientation': -14.49421501159668, 'position': {'x': 8.227262496948242, 'y': 8.894871711730957 }},
{'orientation': 4.713443756103516, 'position': {'x': 10.222247123718262, 'y': 8.883543968200684}},
{'orientation': 172.52919006347656, 'position': {'x': 8.698485374450684, 'y': 8.99336051940918}},
{'orientation': 117.51683807373047, 'position': {'x': 6.665578842163086, 'y': 9.719882011413574}}
]

camera_pose = np.array([
    [1, 0, 0, 0.34],
    [0, 1, 0, 0],
    [0, 0, 1, 0.79],
    [0, 0, 0, 1]
])




def get_robot_position(controller):
    _, data = controller.api.get_robot_status()
    robot_position = data['position']
    print('Robot Location:', data['position'])
    return robot_position

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

def get_object_pose(robot_position, camera_pose, center, rotation_matrix, camera):
    robot_pose = get_pose(robot_position)
    rect = get_bounding_box_xywh(center, 10)
    x, y, depth_val = camera.get_3d_coordinates(rect, center)

    matrix = np.zeros((4, 4))
    matrix[:3, :3] = rotation_matrix
    matrix[:3, 3] = np.array([depth_val, -x, -y])
    matrix[3, 3] = 1
    object_pose = np.round(robot_pose @ (camera_pose @ matrix), 2)
    object_position = object_pose[:3, 3]
    object_rotation = object_pose[:3, :3]
    return object_position, object_rotation, depth_val

def get_angle_to_rotate(robot_position, object_position):
    robot_to_obj_vector = np.array([object_position[0] - robot_position['x'], object_position[1] - robot_position['y']])
    robot_orientation = np.radians(robot_position['orientation'])
    numerator = np.cos(robot_orientation) * robot_to_obj_vector[0] + np.sin(robot_orientation) * robot_to_obj_vector[1]
    denominator = np.sqrt(np.square(robot_to_obj_vector[0]) + np.square(robot_to_obj_vector[1]))
    object_orientation = np.arccos(robot_to_obj_vector[0] / denominator)
    angle = np.degrees(np.arccos(numerator / denominator))
    assert angle >= 0
    if robot_orientation < 0:
        if object_orientation > abs(robot_orientation):
            angle = -angle
    else:
        if object_orientation < abs(robot_orientation):
            angle = -angle
    return angle

def get_next_rotation(rotation_targets, robot_position):
    for object_id in rotation_targets:
        object_position = rotation_targets[object_id]['position']
        distance = np.sqrt(np.sum(np.square(np.array([object_position[0] - robot_position['x'], object_position[1] - robot_position['y']]))))
        angle = get_angle_to_rotate(robot_position, object_position)
        if abs(angle) > 2 and distance < 3 and not rotation_targets[object_id]['done']:
            return object_id, angle

    return None, None

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
    mir_pose = get_mir_pose(x, y, 0.0, to_quaternion(orientation_radians, 0, 0))
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

def get_object_pose3d(position, rotation, object_id) -> ObjectPose3D:
    return ObjectPose3D({'position': position, 'rotation': rotation, 'RefFrameID': '10', 'ObjectID': str(object_id)})

def publish_object_pose(object_position, rotation_matrix, object_id, producer_object_pose):
    
    position = get_point3d(object_position[0], object_position[1], object_position[2])
    rotation = get_rotation(rotation_matrix)
    object_pose = get_object_pose3d(position, rotation, object_id)
    send_pose3d(object_pose, producer_object_pose)


def main():
    
    store = ObjectTrackingStore()
    camera = RealSenseCamera(simulate=False, filepath='./data/bluebox.bag')
    intrinsic = camera.intrinsic
    detector = DopeDetection(intrinsic, './config.yaml')
    controller = MirController()
    waypoint_manager = WaypointManager(WAYPOINTS, controller)

    if PUBLISH:
        producer_object_pose, producer_mir = get_producer()

    rotation_targets = {}

    is_rotating = False
    try:
        while True:

            robot_position = get_robot_position(controller)

            if TIME:
                start_time = time.time()
            image, depth = camera.get_rgb_depth(depth_for_display=False)
            results = detector.process(image, depth)
            if TIME:
                end_time = time.time()
            coordinates, rotations = detector.get_pixel_coordinates(results)

            if DEBUG:
                output = detector.draw_boxes(results, image.copy())

            for object_coordinates, rotation in zip(coordinates, rotations):
                center = tuple(map(int, list(object_coordinates[-1])))

                if (center[0] < 0 or center[1] < 0 or center[0] > intrinsic.width or center[1] > intrinsic.height):
                    continue
                
                print('Euler angles', quaternion_to_euler(rotation))

                rotation_matrix = quaternion_rotation_matrix(rotation)
                object_position, rotation_matrix, depth = get_object_pose(robot_position, camera_pose, center, rotation_matrix, camera)
                object_id, belief_count = store.query_and_push(object_position)


                if DEBUG:
                    draw_object_coords(output, center, object_position[0], object_position[1], object_position[2], object_id)

                if belief_count > 5:
                    if object_id not in rotation_targets:
                        rotation_targets[object_id] = {'angle': get_angle_to_rotate(robot_position, object_position), 'done': False, 'position': object_position}
                    elif not rotation_targets[object_id]['done']:
                        rotation_targets[object_id]['angle'] = get_angle_to_rotate(robot_position, object_position)
                        rotation_targets[object_id]['position'] = object_position

                if belief_count > 3 and PUBLISH:
                    publish_object_pose(object_position, rotation_matrix, object_id, producer_object_pose)


            if PUBLISH:
                publish_mir_pose(robot_position['x'], robot_position['y'], robot_position['orientation'], producer_mir)

            cv2.imshow('Dope Detection', output)
            cv2.waitKey(10)


            if not is_rotating:
                next_to_rotate, angle = get_next_rotation(rotation_targets, robot_position)
                if next_to_rotate is not None:
                    print('Pausing....')
                    waypoint_manager.pause()
                    print('Starting to rotate')
                    controller.rotate(angle)
                    rotation_targets[next_to_rotate]['done'] = True
                    is_rotating = True

            if controller.done:
                if waypoint_manager.paused:
                    is_rotating = False
                    waypoint_manager.unpause()
                else:
                    waypoint_manager.go_to_next_waypoint()

            if waypoint_manager.all_waypoints_reached:
                break
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
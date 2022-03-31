# Algorithm
# Give initial set of waypoints
# While moving,
#  if an object is seen
#    If object is not seen up-close:
#       Pause current waypoint
#       Rotate towards object
#       If object is more that DISTANCE_THRESHOLD away:
#          Go towards object.
#       Continue towards current waypoint
#  Get next waypoint
# 




import time
import numpy as np
import cv2
# from pyquaternion import Quaternion

from object_tracking_store import ObjectTrackingStore
from camera import RealSenseCamera
from detection import DopeDetection
from mir_control.mir_controller import MirController
from kafka.kafka_producer import get_producer, send_pose3d, send_mir_pose
from kafka.AGVPoint3D import AGVPoint3D
from kafka.ObjectPose3D import ObjectPose3D
from kafka.AGVMatrix3x3 import AGVMatrix3x3 
from kafka.MiRPoseStamped import MiRPoseStamped
from kafka.Pose3d import Pose3d
from kafka.TimestampUnix import TimestampUnix

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

class WaypointManager:
    def __init__(self, waypoints, controller):
        self.waypoints = waypoints
        self.controller = controller
        self.current_waypoint_index = -1
        self.paused = False
        self.all_waypoints_reached = False

    def pause(self):
        self.paused = True
        self.controller.cancel()

    def unpause(self):
        self.paused = False
        self.go_to_waypoint(self.current_waypoint_index)

    def go_to_waypoint(self, index):
        self.controller.move_to_position(self.waypoints[index])

    def go_to_next_waypoint(self):
        if self.current_waypoint_index >= len(self.waypoints) - 1:
            self.all_waypoints_reached = True
            return

        self.current_waypoint_index += 1
        self.go_to_waypoint(self.current_waypoint_index)
        



def quaternion_to_euler(quaternion):
    x, y, z, w = quaternion

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    if (abs(sinp) >= 1):
        sign = -1 if sinp < 0 else 1
        pitch = sign * np.pi / 2
    else:
        pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return np.degrees(yaw), np.degrees(pitch), np.degrees(roll)

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

def get_bounding_box_xywh(center, delta):
    return (center[0] - delta, center[1] - delta, 2 * delta, 2 * delta)

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

def draw_object_coords(image, center, x, y, z, object_id):
    cv2.putText(image, 'X: ' + str(round(x, 3)), (center[0] + 20, center[1] + 20), cv2.FONT_HERSHEY_SCRIPT_COMPLEX, 0.5, (255,255,255))
    cv2.putText(image, 'Y: ' + str(round(y, 3)), (center[0] + 20, center[1] + 40), cv2.FONT_HERSHEY_SCRIPT_COMPLEX, 0.5, (255,255,255))
    cv2.putText(image, 'Z: ' + str(round(z, 3)), (center[0] + 20, center[1] + 60), cv2.FONT_HERSHEY_SCRIPT_COMPLEX, 0.5, (255,255,255))
    cv2.putText(image, 'ID: ' + str(object_id), (center[0] + 20, center[1] + 80), cv2.FONT_HERSHEY_SCRIPT_COMPLEX, 0.5, (255,255,255))

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
        # elif abs(angle) <= 2:
        #     rotation_targets[object_id]['done'] = True
    return None, None

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

def get_object_pose3d(position, rotation, object_id) -> ObjectPose3D:
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
                # q = Quaternion(x = rotation[0], y = rotation[1], z = rotation[2], w = rotation[3])
                # print('Rotation axis:', q.axis)
                # print('Euler angles from library:', q.degrees)
                rotation_matrix = quaternion_rotation_matrix(rotation)
                object_position, rotation_matrix, depth = get_object_pose(robot_position, camera_pose, center, rotation_matrix, camera)
                object_id, belief_count = store.query_and_push(object_position)
                # print('Object: ', object_id, 'Position:', object_position)

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


            # if PUBLISH:
            #     publish_mir_pose(robot_position['x'], robot_position['y'], robot_position['orientation'], producer_mir)

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


            print('Rotation targets', rotation_targets)
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
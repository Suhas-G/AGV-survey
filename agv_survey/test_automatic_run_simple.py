import cv2
from detection import CameraIntrinsic, DopeDetection
from camera import RealSenseCamera
import numpy as np

from pathlib import Path
from matplotlib import pyplot as plt

from mir_control.mir_api import MirRestApi
from mir_control.mir_controller import MirController
from object_tracking_store import ObjectTrackingStore
from time import sleep

DEBUG = True

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






# {'orientation': 47.47665786743164,
#  'x': 10.593080520629883,
#  'y': 9.14310073852539}

# GOALS = [[{'position': {'x': 10.426, 'y': 7.042}, 'orientation': -176.427}], 
#         [{'position': {'x': 4.804, 'y': 7.636}, 'orientation': -38.781}]]

GOALS = [[{'orientation': 1.2928093671798706,
 'position': {'x': 7.895124435424805, 'y': 8.918671607971191}
 }],
 [{'orientation': -28.040300369262695,
 'position': {'x': 10.420890808105469, 'y': 8.547136306762695}
 }]]


def get_object_pose(robot_pose, camera_pose, center, camera):
    rect = get_bounding_box_xywh(center, 10)
    x, y, depth_val = camera.get_3d_coordinates(rect, center)

    object_position = robot_pose @ (camera_pose @ np.array([depth_val, -x, -y, 1]))

    return object_position[:-1]

def main():
    goal_index = 0
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

    controller = None


    try:
        while True:
            _, data = api.get_robot_status()
            robot_position = data['position']
            robot_pose = get_pose(robot_position)
            print('Robot Location:', data['position'])


            image, depth = camera.get_rgb_depth(depth_for_display=False)
            results = detector.process(image, depth)
            coordinates, rotations = detector.get_pixel_coordinates(results)

            if DEBUG:
                output = detector.draw_boxes(results, image.copy())

            for object_coordinates, rotation in zip(coordinates, rotations):
                center = tuple(map(int, list(object_coordinates[-1])))

                if (center[0] < 0 or center[1] < 0 or center[0] > intrinsic.width or center[1] > intrinsic.height):
                    continue
                
                object_position = get_object_pose(robot_pose, camera_pose, center, camera)
                object_id, _ = store.query_and_push(object_position)
                print('Object: ', object_id, 'Position:', object_position)

                if DEBUG:
                    # draw_object_center(output, center, rect)
                    # draw_object_position(output, center, x, y, depth_val)
                    draw_object_coords(output, center, object_position[0], object_position[1], object_position[2], object_id)

            # rp_mag = np.sqrt(np.square([data['position']['x'], data['position']['y']]).sum())
            # op_mag = np.sqrt(np.square(object_position[0:2]).sum())
            # orientation = ((data['position']['orientation'] *np.pi/180) + 
            #                 (np.arccos((data['position']['x'] * robot_pose[0] + data['position']['y'] * robot_pose[1])/(rp_mag * op_mag))))
            # api.add_move_to_position_action(api.mission_id, data['position']['x'], data['position']['y'], data['position']['orientation'] + orientation)
            
            cv2.imshow('Dope Detection', output)
            cv2.waitKey(10)

            if controller is None:
                controller = MirController(GOALS[goal_index])
                goal_index += 1

            if controller.is_done() and goal_index < len(GOALS):
                controller.stop()
                controller = MirController(GOALS[goal_index])
                goal_index += 1
                # goal_index = (goal_index + 1) % len(GOALS)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
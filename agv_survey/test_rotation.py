import cv2
from detection import CameraIntrinsic, DopeDetection
from camera import RealSenseCamera
import numpy as np

from mir_control.mir_controller import MirController
from object_tracking_store import ObjectTrackingStore

DEBUG = True

ROTATION_TARGETS = {}

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

def get_object_pose(robot_pose, camera_pose, center, camera):
    rect = get_bounding_box_xywh(center, 10)
    x, y, depth_val = camera.get_3d_coordinates(rect, center)

    object_position = robot_pose @ (camera_pose @ np.array([depth_val, -x, -y, 1]))

    return object_position[:-1]

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

    # if robot_position['y'] > object_position[1]:
    #     if robot_position['x'] > object_position[0]:
    #         angle = -abs(angle)
    # else:
    #     if robot_position['x'] > object_position[0]:
    #         angle = -abs(angle)
    return angle

def main():
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

    controller = MirController()

    try:
        while True:
            _, data = controller.api.get_robot_status()
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
                object_id, count = store.query_and_push(object_position)
                print('Object: ', object_id, 'Position:', object_position)

                if DEBUG:
                    draw_object_coords(output, center, object_position[0], object_position[1], object_position[2], object_id)

                if count > 5 and object_id not in ROTATION_TARGETS:
                    # robot_orientation = robot_position['orientation']
                    # if robot_position['y'] > object_position[1]:
                    #     robot_orientation = -robot_position['orientation']
                    ROTATION_TARGETS[object_id] = {
                        'angle': get_angle_to_rotate(robot_position, object_position),
                        'done': False
                    }
                elif object_id in ROTATION_TARGETS and not  ROTATION_TARGETS[object_id]['done']:
                    ROTATION_TARGETS[object_id] = {
                        'angle': get_angle_to_rotate(robot_position, object_position),
                        'done': False
                    }


            cv2.imshow('Dope Detection', output)
            cv2.waitKey(10)
            print(ROTATION_TARGETS)
            # print('Controller is done:', controller.done)
            if controller.done:
                for object_id in ROTATION_TARGETS:
                    if not ROTATION_TARGETS[object_id]['done'] and abs(ROTATION_TARGETS[object_id]['angle']) > 5:
                        print('Starting to rotate:', object_id, ROTATION_TARGETS[object_id]['angle'])
                        controller.rotate(ROTATION_TARGETS[object_id]['angle'])
                        ROTATION_TARGETS[object_id]['done'] = True
                        break


    except KeyboardInterrupt:
        pass


def test():
    controller = MirController()
    # angles = [60, -60, 90, 30, -30, 0]
    while True:
        angle = float(input())
        print('Rotating:', angle)
        controller.rotate(angle)


if __name__ == '__main__':
    main()
    # test()
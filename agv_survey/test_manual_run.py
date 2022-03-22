import cv2
from detection import CameraIntrinsic, DopeDetection
from camera import RealSenseCamera
import numpy as np

from pathlib import Path
from matplotlib import pyplot as plt

from mir_control.mir_api import MirRestApi
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

    try:
        while True:
            # input()
            # sleep(5)
            _, data = api.get_robot_status()
            robot_pose = get_pose(data['position'])
            print('Robot Location:', data['position'])
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
                object_id = store.query_and_push(object_position[:-1])
                print('Object: ', object_id, 'Position:', object_position)

                if DEBUG:
                    draw_object_center(output, center, rect)
                    # draw_object_position(output, center, x, y, depth_val)
                    draw_object_coords(output, center, object_position[0], object_position[1], object_position[2], object_id)

            

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
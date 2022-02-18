import sys
import os

sys.path.insert(1, os.path.abspath("./CenterPose/src"))

import cv2
from camera import RealSenseCamera
from detection import MediaPipeDetection, CameraIntrinsic, CenterPoseDetection

DEMO_FILE = './data/shoe.bag'


def get_bounding_box_xywh(center, delta):
    return (center[0] - delta, center[1] - delta, 2 * delta, 2 * delta)

def draw_object_center(image, center, rect):
    cv2.circle(image, center, 3, [255, 255, 0], -1)
    cv2.rectangle(image, (rect[0], rect[1]), (rect[0] + rect[2], rect[1] + rect[3]), [255, 0, 255], 2)

def draw_object_position(image, center, x, y, depth):
    cv2.putText(image, str(depth), (center[0] + 20, center[1] + 20), cv2.FONT_HERSHEY_SCRIPT_COMPLEX, 0.5, (255,255,255))
    cv2.putText(image, str(x), (center[0] + 20, center[1] + 40), cv2.FONT_HERSHEY_SCRIPT_COMPLEX, 0.5, (255,255,255))
    cv2.putText(image, str(y), (center[0] + 20, center[1] + 60), cv2.FONT_HERSHEY_SCRIPT_COMPLEX, 0.5, (255,255,255))


def main():
    try:
        camera = RealSenseCamera(simulate=True, filepath=DEMO_FILE)
        intrinsic = camera.intrinsic
        # detector = MediaPipeDetection(CameraIntrinsic(intrinsic.width, intrinsic.height, 
        #                                 intrinsic.fx, intrinsic.fy, intrinsic.ppx, intrinsic.ppy))
        detector = CenterPoseDetection(CameraIntrinsic(intrinsic.width, intrinsic.height, 
                                        intrinsic.fx, intrinsic.fy, intrinsic.ppx, intrinsic.ppy))
        while True:
            image, depth = camera.get_rgb_depth(depth_for_display=False)
            results = detector.process(image, depth)
            # output = detector.draw_boxes(results, image.copy())
            output = image.copy()
            coordinates, rotations = detector.get_pixel_coordinates(results)

            for object_coordinates, rotation in zip(coordinates, rotations):
                coordinate = object_coordinates[0]
                center = tuple(map(int, coordinate))
                if (center[0] > image.shape[0] or center[1] > image.shape[1]):
                        continue
                rect = get_bounding_box_xywh(center, 10)
                x, y, depth_val = camera.get_3d_coordinates(rect, center)

                print('X', x, 'Y', y, 'Depth', depth_val)

                draw_object_center(output, center, rect)
                draw_object_position(output, center, x, y, depth_val)

            cv2.imshow('Detected', cv2.cvtColor(output, cv2.COLOR_RGB2BGR))
            cv2.waitKey(10)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
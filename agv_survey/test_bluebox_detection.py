import cv2
from detection import CameraIntrinsic, DopeDetection
from camera import RealSenseCamera
import numpy as np

from pathlib import Path
from matplotlib import pyplot as plt

DEBUG = True

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
    camera = RealSenseCamera(simulate=False, filepath='./data/bluebox.bag')
    intrinsic = camera.intrinsic
    print(intrinsic)
    detector = DopeDetection(intrinsic, './config.yaml')

    try:
        while True:
            image, depth = camera.get_rgb_depth(depth_for_display=False)
            # image = cv2.resize(cv2.cvtColor(image, cv2.COLOR_BGR2RGB), (640, 480))
            results = detector.process(image, depth)
            print('Detected Objects:', len(results))
            coordinates, rotations = detector.get_pixel_coordinates(results)
            if DEBUG:
                output = detector.draw_boxes(results, image.copy())
            for object_coordinates, rotation in zip(coordinates, rotations):
                center = tuple(map(int, list(object_coordinates[-1])))
                print(center, image.shape)
                if (center[0] > image.shape[0] or center[1] > image.shape[1]):
                    continue
                rect = get_bounding_box_xywh(center, 10)
                x, y, depth_val = camera.get_3d_coordinates(rect, center)

                if DEBUG:
                    draw_object_center(output, center, rect)
                    draw_object_position(output, center, x, y, depth_val)

            

            cv2.imshow('Dope Detection', output)
            # detector.plot_belief_map(image)
            if len(results) > 0:
                cv2.waitKey(0)
            else:
                cv2.waitKey(10)
            # plt.close()
    except KeyboardInterrupt:
        pass

def test_simulated_images():
    camera_intrinsic = CameraIntrinsic(640, 480, 391, 391, 291, 240.0)
    detector = DopeDetection(camera_intrinsic, './config.yaml')

    files = Path('./data/test_images').glob('*.png')

    for file in files:
        image = cv2.imread(str(file))
        image = cv2.resize(cv2.cvtColor(image, cv2.COLOR_BGR2RGB), (640, 480))
        results = detector.process(image, None)
        print('Detected Objects:', len(results))
        output = detector.draw_boxes(results, image.copy())

        cv2.imshow('Dope Detection', output)
        detector.plot_belief_map(image)
        cv2.waitKey(0)



if __name__ == '__main__':
    main()
    # test_simulated_images()
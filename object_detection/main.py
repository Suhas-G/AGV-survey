import cv2
from detection.detection import MediaPipeDetection
from view.camera import RealSenseCamera, ZEDCamera


def main():
    try:
        # camera = ZEDCamera()
        camera = RealSenseCamera()
        detector = MediaPipeDetection()
        print('Starting detection...')
        while True:
            image, depth = camera.get_rgb_depth(depth_for_display=False)
            results = detector.process(image, depth)
            output = detector.draw_boxes(results, image.copy())
            cv2.imshow('MediaPipe Objectron', cv2.resize(output, (640, 360)))
            cv2.waitKey(10)

    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()

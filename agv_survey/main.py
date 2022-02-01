import cv2
from detection import CameraIntrinsic, MediaPipeDetection
from camera import RealSenseCamera
import numpy as np

from kafka.kafka_producer import get_producer, send_pose3d
from kafka.AGVPoint3D import AGVPoint3D
from kafka.ObjectPose3D import ObjectPose3D
from kafka.AGVMatrix3x3 import AGVMatrix3x3

from mir_control.mir_controller import MirController
from object_tracking_store import ObjectTrackingStore

DEBUG = True
PUBLISH = True


camera_pose = np.array([
    [1, 0, 0, 0.34],
    [0, 1, 0, 0],
    [0, 0, 1, 0.79],
    [0, 0, 0, 1]
])


def get_point3d(x, y, z) -> AGVPoint3D:
    return AGVPoint3D({'x': float(x), 'y': float(y), 'z': float(z)})

def get_rotation(rotation_matrix) -> AGVMatrix3x3:
    data = {}
    for i in range(len(rotation_matrix)):
        for j in range(len(rotation_matrix[i])):
            data['m{}{}'.format(i, j)] = float(rotation_matrix[i, j])
    return AGVMatrix3x3(data)

def get_object_pose(position, rotation, object_id) -> ObjectPose3D:
    return ObjectPose3D({'position': position, 'rotation': rotation, 'RefFrameID': '0', 'ObjectID': str(object_id)})

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
        store = ObjectTrackingStore()
        camera = RealSenseCamera(simulate=False)
        intrinsic = camera.intrinsic
        detector = MediaPipeDetection(CameraIntrinsic(intrinsic.width, intrinsic.height, 
                                        intrinsic.fx, intrinsic.fy, intrinsic.ppx, intrinsic.ppy))
        
        if PUBLISH:
            producer = get_producer()

        controller = MirController([
                {'position': {'x': 10.426, 'y': 7.042}, 'orientation': -176.427},
                {'position': {'x': 4.804, 'y': 7.636}, 'orientation': -38.781}
        ])

        while True:
            image, depth = camera.get_rgb_depth(depth_for_display=False)
            
            results = detector.process(image, depth)
            coordinates, rotations = detector.get_pixel_coordinates(results)

            if DEBUG:
                output = detector.draw_boxes(results, image.copy())

            for object_coordinates, rotation in zip(coordinates, rotations):
                coordinate = object_coordinates[0]
                center = tuple(map(int, coordinate))
                if (center[0] > image.shape[0] or center[1] > image.shape[1]):
                        continue
                rect = get_bounding_box_xywh(center, 10)
                x, y, depth_val = camera.get_3d_coordinates(rect, center)

                if DEBUG:
                    draw_object_center(output, center, rect)
                    draw_object_position(output, center, x, y, depth_val)

                robot_pose = controller.get_robot_pose()
                object_position = robot_pose @ (camera_pose @ np.array([depth_val, x, -y, 1]))
                object_id = store.query_and_push(object_position[:-1])
                position = get_point3d(object_position[0], object_position[1], object_position[2])
                rotation = get_rotation(rotation)
                object_pose = get_object_pose(position, rotation, object_id)
                if PUBLISH:
                    send_pose3d(object_pose, producer)

            if DEBUG:
                cv2.imshow('MediaPipe Objectron', cv2.cvtColor(output, cv2.COLOR_RGB2BGR))
                cv2.waitKey(10)

    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
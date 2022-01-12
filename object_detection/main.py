import cv2
from detection.detection import CameraIntrinsic, MediaPipeDetection
from view.camera import RealSenseCamera
import numpy as np

from publisher.producer import get_producer, send_pose3d
from publisher.avro_obj.AGVPoint3D import AGVPoint3D
from publisher.avro_obj.ObjectPose3D import ObjectPose3D
from publisher.avro_obj.AGVMatrix3x3 import AGVMatrix3x3



def get_point3d(x, y, z) -> AGVPoint3D:
    return AGVPoint3D({'x': float(x), 'y': float(y), 'z': float(z)})

def get_rotation(rotation_matrix) -> AGVMatrix3x3:
    data = {}
    for i in range(len(rotation_matrix)):
        for j in range(len(rotation_matrix[i])):
            data['m{}{}'.format(i, j)] = float(rotation_matrix[i, j])
    return AGVMatrix3x3(data)

def get_object_pose(position, rotation) -> ObjectPose3D:
    return ObjectPose3D({'position': position, 'rotation': rotation, 'RefFrameID': '0', 'ObjectID': '0'})

def get_bounding_box_xywh(center, delta):
    return (center[0] - delta, center[1] - delta, 2 * delta, 2 * delta)

def main():
    try:
        # camera = ZEDCamera()
        camera = RealSenseCamera(simulate=False)
        intrinsic = camera.intrinsic
        detector = MediaPipeDetection(CameraIntrinsic(intrinsic.width, intrinsic.height, 
                                        intrinsic.fx, intrinsic.fy, intrinsic.ppx, intrinsic.ppy))
        producer = get_producer()
        print('Starting detection...')
        while True:
            image, depth = camera.get_rgb_depth(depth_for_display=False)
            
            results = detector.process(image, depth)
            coordinates, rotations = detector.get_pixel_coordinates(results)
            output = detector.draw_boxes(results, image.copy())
            for object_coordinates, rotation in zip(coordinates, rotations):
                
                for i, coordinate in enumerate(object_coordinates):
                    center = tuple(map(int, coordinate))
                    cv2.circle(output, center, 3, [255, 255, 0], -1)
                    if i == 0:
                        rect = get_bounding_box_xywh(center, 10)
                        x, y, depth_val = camera.get_3d_coordinates(rect, center)
                        
                        # depth_val, _, _, _ = cv2.mean(depth[center[1] - 5: center[1] + 5, center[0] - 5: center[0] + 5])
                        # print('Depth', depth)
                        
                        cv2.rectangle(output, (rect[0], rect[1]), (rect[0] + rect[2], rect[1] + rect[3]), [255, 0, 255], 2)
                        cv2.putText(output, str(depth_val), (center[0] + 20, center[1] + 20), cv2.FONT_HERSHEY_SCRIPT_COMPLEX, 0.5, (255,255,255))
                        cv2.putText(output, str(x), (center[0] + 20, center[1] + 40), cv2.FONT_HERSHEY_SCRIPT_COMPLEX, 0.5, (255,255,255))
                        cv2.putText(output, str(y), (center[0] + 20, center[1] + 60), cv2.FONT_HERSHEY_SCRIPT_COMPLEX, 0.5, (255,255,255))

                        position = get_point3d(x, y, depth_val)
                        rotation = get_rotation(rotation)
                        object_pose = get_object_pose(position, rotation)
                        # print(object_pose.serialize())
                        send_pose3d(object_pose, producer)
                        
            cv2.imshow('MediaPipe Objectron', cv2.cvtColor(output, cv2.COLOR_RGB2BGR))
            # cv2.imshow('Stream', np.hstack((image, depth)))
            cv2.waitKey(10)

    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()

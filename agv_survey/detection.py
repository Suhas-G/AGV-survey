import cv2
import mediapipe as mp

mpd = mp.solutions.drawing_utils
mpo = mp.solutions.objectron


MODEL_NAME = 'Shoe'


class CameraIntrinsic:
    def __init__(self, width, height, fx, fy, ppx, ppy) -> None:
        self.width = width
        self.height = height
        self.fx = fx
        self.fy = fy
        self.ppx = ppx
        self.ppy = ppy




class MediaPipeDetection:
    def __init__(self, camera_intrinsic: CameraIntrinsic) -> None:
        super().__init__()
        self.camera_intrinsic = camera_intrinsic
        self.objectrons = [mpo.Objectron(static_image_mode=False,
                                max_num_objects=2,
                                min_detection_confidence=0.4,
                                min_tracking_confidence=0.95,
                                model_name=MODEL_NAME,
                                focal_length=(self.camera_intrinsic.fx, self.camera_intrinsic.fy),
                                principal_point=(self.camera_intrinsic.ppx, self.camera_intrinsic.ppy),
                                image_size=(self.camera_intrinsic.width, self.camera_intrinsic.height)
                                )
                            ]

    def process(self, image, depth):
        image.flags.writeable = False
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        results = []
        for objectron in self.objectrons:
            result = objectron.process(image)
            results.append(result)

        image.flags.writeable = True
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        return results


    def draw_boxes(self, results, image):
        for result in results:
            if result.detected_objects:
                
                for detected_object in result.detected_objects:
                    mpd.draw_landmarks(
                    image, detected_object.landmarks_2d, mpo.BOX_CONNECTIONS)
                    mpd.draw_axis(image, detected_object.rotation,
                                        detected_object.translation)
        return image #cv2.flip(image, 1)

    def get_pixel_coordinates(self, results):
        objects = []
        rotations = []
        for result in results:
            if result.detected_objects:
                for detected_object in result.detected_objects:
                    pixels = []
                    for landmark in detected_object.landmarks_2d.landmark:
                        x_pixel = landmark.x * self.camera_intrinsic.width
                        y_pixel = landmark.y * self.camera_intrinsic.height
                        pixels.append((x_pixel, y_pixel))
                    objects.append(pixels)
                    rotations.append(detected_object.rotation)
        return objects, rotations


    def __del__(self):
        for objectron in self.objectrons:
            objectron.close()


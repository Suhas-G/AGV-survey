from abc import ABC, abstractmethod
import numpy.typing as npt
import mediapipe as mp
import cv2
mpd = mp.solutions.drawing_utils
mpo = mp.solutions.objectron


class Detection(ABC):
    def __init__(self) -> None:
        super().__init__()

    @abstractmethod
    def process(self, image: npt.NDArray, depth: npt.NDArray):
        pass

    @abstractmethod
    def draw_boxes(results):
        pass

class MediaPipeDetection(Detection):
    def __init__(self) -> None:
        super().__init__()
        self.objectrons = [mpo.Objectron(static_image_mode=False,
                                max_num_objects=2,
                                min_detection_confidence=0.4,
                                min_tracking_confidence=0.95,
                                model_name='Shoe'),
                            mpo.Objectron(static_image_mode=False,
                                max_num_objects=2,
                                min_detection_confidence=0.4,
                                min_tracking_confidence=0.95,
                                model_name='Chair'),
                            mpo.Objectron(static_image_mode=False,
                                max_num_objects=2,
                                min_detection_confidence=0.4,
                                min_tracking_confidence=0.95,
                                model_name='Cup')
                            ]

    def process(self, image: npt.NDArray, depth: npt.NDArray):
        image.flags.writeable = False
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        results = []
        for objectron in self.objectrons:
            result = objectron.process(image)
            results.append(result)

        image.flags.writeable = True
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        return results


    def draw_boxes(self, results, image) -> npt.NDArray:
        for result in results:
            if result.detected_objects:
                
                for detected_object in result.detected_objects:
                    mpd.draw_landmarks(
                    image, detected_object.landmarks_2d, mpo.BOX_CONNECTIONS)
                    mpd.draw_axis(image, detected_object.rotation,
                                        detected_object.translation)
        return cv2.flip(image, 1)


    def __del__(self):
        for objectron in self.objectrons:
            objectron.close()


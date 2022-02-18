import cv2
import mediapipe as mp
import numpy as np

mpd = mp.solutions.drawing_utils
mpo = mp.solutions.objectron


from CenterPose.src.lib.detectors.object_pose import ObjectPoseDetector
from CenterPose.src.lib.opts import opts

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

class CenterPoseDetection:
    def __init__(self, camera_intrinsic: CameraIntrinsic) -> None:
        self.camera_intrinsic = camera_intrinsic
        self.opt, self.meta = self.initialise_options()
        self.detector = ObjectPoseDetector(self.opt)
        self.detector.pause = False

    def initialise_options(self):
        opt = opts().parser.parse_args()
        opt.arch = 'dlav1_34'
        # Default setting
        opt.nms = True
        opt.obj_scale = True
        # opt.tracking_task = True
        # opt.pre_img = True
        # opt.pre_hm = True
        # opt.tracking = True
        # opt.pre_hm_hp = True
        # opt.tracking_hp = True
        # opt.track_thresh = 0.1

        # opt.obj_scale_uncertainty = True
        # opt.hps_uncertainty = True
        # opt.kalman = True
        # opt.scale_pool = True

        # opt.vis_thresh = max(opt.track_thresh, opt.vis_thresh)
        # opt.pre_thresh = max(opt.track_thresh, opt.pre_thresh)
        # opt.new_thresh = max(opt.track_thresh, opt.new_thresh)

        meta = {}

        meta['camera_matrix'] = np.array(
            [[self.camera_intrinsic.fx, 0, self.camera_intrinsic.ppx],
                [0, self.camera_intrinsic.fy, self.camera_intrinsic.ppy],
                [0, 0, 1]
            ])
        opt.cam_intrinsic = meta['camera_matrix']

        opt.use_pnp = True

        opt.debug = max(opt.debug, 1)

        # Update default configurations
        opt = opts().parse(opt)

        # Update dataset info/training params
        opt = opts().init(opt)
        return opt, meta
        

    def process(self, image, depth):
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        results = self.detector.run(image, meta_inp=self.meta, filename='data/test.png')
        return results

    def draw_boxes(self, results, image):
        pass

    def get_pixel_coordinates(self, results):
        objects = []
        for obj in results['objects']:
            pixels = []
            for landmark in obj['kps_pnp']:
                x_pixel = landmark[0] * self.camera_intrinsic.width
                y_pixel = landmark[1] * self.camera_intrinsic.height
                pixels.append((x_pixel, y_pixel))
            objects.append(pixels)
        return objects, [None for _ in range(len(objects))]
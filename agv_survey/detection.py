import cv2
import mediapipe as mp
import yaml
import numpy as np
mpd = mp.solutions.drawing_utils
mpo = mp.solutions.objectron

from inference.detector import ModelData, ObjectDetector
from inference.cuboid_pnp_solver import CuboidPNPSolver
from inference.cuboid import Cuboid3d

from utils import DrawCube
from PIL import ImageDraw, Image

from torch.autograd import Variable
from torchvision import transforms
from matplotlib import pyplot as plt

transform = transforms.Compose([
    # transforms.Scale(IMAGE_SIZE),
    # transforms.CenterCrop((imagesize,imagesize)),
    transforms.ToTensor(),
    # transforms.Normalize((0.5, 0.5, 0.5), (0.5, 0.5, 0.5)),
    transforms.Normalize(    
                    (0.485, 0.456, 0.406),
                    (0.229, 0.224, 0.225)
                    )
    ])

MODEL_NAME = 'Shoe'


def viz_layer(layer, n_filters=9):
    fig = plt.figure(figsize=(20, 20))
    for i in range(n_filters):
        ax = fig.add_subplot(4, 5, i + 1, xticks=[], yticks=[])
        # grab layer outputs
        ax.imshow(np.squeeze(layer[i].data.numpy()), cmap='gray')
        ax.set_title('Output %s' % str(i + 1))

class CameraIntrinsic:
    def __init__(self, width, height, fx, fy, ppx, ppy) -> None:
        self.width = width
        self.height = height
        self.fx = fx
        self.fy = fy
        self.ppx = ppx
        self.ppy = ppy

    def __str__(self) -> str:
        return str(self.__dict__)




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

class DopeDetection:
    def __init__(self, camera_intrinsic, config_file) -> None:
        self.camera_intrinsics = camera_intrinsic
        (self.weights, self.draw_colors, self.dimensions, self.matrix_camera, self.dist_coeffs, 
                                                        self.config_detect) = self.read_config(config_file)
        self.models, self.pnp_solvers = self.load_models_solvers(self.weights, self.dimensions, 
                                                            self.matrix_camera, self.dist_coeffs)
    def process(self, image, depth):
        for m in self.models:
        # Detect object
            self.draw_colors[m] = tuple(self.draw_colors[m])
            try:
                results, _ = ObjectDetector.detect_object_in_image(
                    self.models[m].net,
                    self.pnp_solvers[m],
                    image,
                    self.config_detect
                )
            except:
                results = []

        return results


    def draw_boxes(self, results, image):
        im = Image.fromarray(image)
        draw = ImageDraw.Draw(im)
        for i_r, result in enumerate(results):
            if result["location"] is None:
                continue
            # loc = result["location"]
            # ori = result["quaternion"]

            # Draw the cube
            if None not in result['projected_points']:
                points2d = []
                for pair in result['projected_points']:
                    points2d.append(tuple(pair))
                DrawCube(points2d, draw=draw)
        return cv2.cvtColor(np.array(im), cv2.COLOR_RGB2BGR)

    def get_pixel_coordinates(self, results):
        rotations = []
        coordinates = []
        for result in results:
            rotations.append(result['quaternion'])
            coordinates.append(result['projected_points'])

        return coordinates, rotations

    def read_config(self, yaml_path):
        weights = {}
        draw_colors = {}
        dimensions = {}
        matrix_camera = np.zeros((3,3))
        dist_coeffs = np.zeros((4,1))
        config_detect = lambda: None
        with open(yaml_path, 'r') as stream:
            try:
                print("Loading DOPE parameters from '{}'...".format(yaml_path))
                params = yaml.full_load(stream)
                print('    Parameters loaded.')
            except yaml.YAMLError as exc:
                print(exc)


            weights = params['weights']
            draw_colors = params['draw_colors']
            dimensions = params['dimensions']

            # Initialize parameters
            matrix_camera[0,0] = self.camera_intrinsics.fx
            matrix_camera[1,1] = self.camera_intrinsics.fy
            matrix_camera[0,2] = self.camera_intrinsics.ppx
            matrix_camera[1,2] = self.camera_intrinsics.ppy
            matrix_camera[2,2] = 1

            # matrix_camera[0,0] = params['camera_settings']['fx']
            # matrix_camera[1,1] = params['camera_settings']['fy']
            # matrix_camera[0,2] = params['camera_settings']['cx']
            # matrix_camera[1,2] = params['camera_settings']['cy']
            # matrix_camera[2,2] = 1
            
            # print('Camera Matrix: ', matrix_camera)
            # print('Dimensions: ', dimensions)

            if "dist_coeffs" in params["camera_settings"]:
                dist_coeffs = np.array(params["camera_settings"]['dist_coeffs'])
            
            config_detect.mask_edges = 1
            config_detect.mask_faces = 1
            config_detect.vertex = 1
            config_detect.threshold = 0.5
            config_detect.softmax = 1000
            config_detect.thresh_angle = params['thresh_angle']
            config_detect.thresh_map = params['thresh_map']
            config_detect.sigma = params['sigma']
            config_detect.thresh_points = params["thresh_points"]

        return weights, draw_colors, dimensions, matrix_camera, dist_coeffs, config_detect

    def load_models_solvers(self, weights, dimensions, matrix_camera, dist_coeffs):

        models = {}
        pnp_solvers = {}


        for model in weights:
            models[model] = \
                ModelData(
                    model,
                    weights[model]
                )
            models[model].load_net_model()


            pnp_solvers[model] = \
                CuboidPNPSolver(
                    model,
                    matrix_camera,
                    Cuboid3d(dimensions[model]),
                    dist_coeffs=dist_coeffs
                )

        return models, pnp_solvers


    def plot_belief_map(self, image):

        for m in self.models:
            net_model = self.models[m].net
            image_tensor = transform(image.copy())
            image_torch = Variable(image_tensor).cuda().unsqueeze(0)
            out, seg = net_model(image_torch)

            vertex2 = out[-1][0].cpu()
            aff = seg[-1][0].cpu()

            viz_layer(vertex2)
            viz_layer(aff, n_filters=16)

            plt.show()
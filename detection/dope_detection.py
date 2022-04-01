
import cv2
import numpy as np
import yaml
from PIL import Image, ImageDraw

from .cuboid import Cuboid3d
from .cuboid_pnp_solver import CuboidPNPSolver
from .detector import ModelData, ObjectDetector
from .utils import CameraIntrinsic, DrawCube


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

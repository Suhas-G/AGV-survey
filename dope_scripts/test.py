import numpy as np
import yaml

from inference.detector import ModelData, ObjectDetector
from inference.cuboid_pnp_solver import CuboidPNPSolver
from inference.cuboid import Cuboid3d

from utils import DrawCube
from PIL import ImageDraw, Image

from pathlib import Path
import cv2

IMAGES = './images/'
CONFIG_FILE = './config.yaml'


def read_config(yaml_path):
    weights = {}
    draw_colors = {}
    dimensions = {}
    matrix_camera = np.zeros((3,3))
    dist_coeffs = np.zeros((4,1))
    config_detect = lambda: None
    with open(yaml_path, 'r') as stream:
        try:
            print("Loading DOPE parameters from '{}'...".format(yaml_path))
            params = yaml.load(stream)
            print('    Parameters loaded.')
        except yaml.YAMLError as exc:
            print(exc)


        weights = params['weights']
        draw_colors = params['draw_colors']
        dimensions = params['dimensions']

        # Initialize parameters
        matrix_camera[0,0] = params["camera_settings"]['fx']
        matrix_camera[1,1] = params["camera_settings"]['fy']
        matrix_camera[0,2] = params["camera_settings"]['cx']
        matrix_camera[1,2] = params["camera_settings"]['cy']
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

def load_models_solvers(weights, dimensions, matrix_camera, dist_coeffs):

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


def detect_objects(in_img, models, pnp_solvers, config_detect, draw_colors, draw):


    for m in models:
        # Detect object
        draw_colors[m] = tuple(draw_colors[m])
        try:
            results, _ = ObjectDetector.detect_object_in_image(
                models[m].net,
                pnp_solvers[m],
                in_img,
                config_detect
            )
        except:
            results = []

        print('Detected Objects:', len(results))
        # Overlay cube on image
        for i_r, result in enumerate(results):
            if result["location"] is None:
                continue
            loc = result["location"]
            ori = result["quaternion"]

            # Draw the cube
            if None not in result['projected_points']:
                points2d = []
                for pair in result['projected_points']:
                    points2d.append(tuple(pair))
                DrawCube(points2d, draw=draw)


def main():
    files = Path(IMAGES).glob('*.png')
    # print(list(files))
    weights, draw_colors, dimensions, matrix_camera, dist_coeffs, config_detect = read_config(CONFIG_FILE)
    models, pnp_solvers = load_models_solvers(weights, dimensions, matrix_camera, dist_coeffs)

    print(draw_colors, weights)
    for file in files:
        in_img = cv2.imread(str(file))
        in_img = cv2.resize(in_img, (640, 480))
        in_img = cv2.cvtColor(in_img, cv2.COLOR_BGR2RGB)

        im = Image.fromarray(in_img)
        draw = ImageDraw.Draw(im)
        detect_objects(in_img, models, pnp_solvers, config_detect, draw_colors, draw)

        cv2.imshow('output', cv2.cvtColor(np.array(im), cv2.COLOR_RGB2BGR))
        cv2.waitKey(0)




if __name__ == '__main__':
    main()
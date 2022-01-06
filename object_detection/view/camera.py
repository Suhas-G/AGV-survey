from abc import ABC, abstractmethod
from typing import Tuple

import pyzed.sl as sl
import numpy as np
import numpy.typing as npt

import pyrealsense2 as rs



class Camera(ABC):
    def __init__(self, simulate = False) -> None:
        super().__init__()
        self.simulate = simulate

    @abstractmethod
    def get_rgb_depth(self):
        pass



class ZEDCamera(Camera):
    def __init__(self, simulate=False) -> None:
        super().__init__(simulate=simulate)
        self.zed = sl.Camera()
        # Create a InitParameters object and set configuration parameters
        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.HD1080  # Use HD1080 video mode
        init_params.camera_fps = 30  # Set fps at 30

        # Open the camera
        err = self.zed.open(init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            exit(1)

        print('ZED Camera opened')
        camera_resolution = self.zed.get_camera_information().camera_resolution
        self.image_zed = sl.Mat(camera_resolution.width, camera_resolution.height, sl.MAT_TYPE.U8_C3)
        self.depth_zed = sl.Mat(camera_resolution.width, camera_resolution.height)
        self.runtime_parameters = sl.RuntimeParameters()

    def get_rgb_depth(self, depth_for_display = False) -> Tuple[npt.NDArray, npt.NDArray]:
        if self.zed.grab(self.runtime_parameters) == sl.ERROR_CODE.SUCCESS:
            self.zed.retrieve_image(self.image_zed, sl.VIEW.LEFT)
            if depth_for_display:
                self.zed.retrieve_image(self.depth_zed, sl.VIEW.DEPTH)
            else:
                self.zed.retrieve_measure(self.depth_zed, sl.MEASURE.DEPTH)

        return self.image_zed.get_data()[:, :, :-1], self.depth_zed.get_data()




class RealSenseCamera(Camera):
    def __init__(self, simulate=False) -> None:
        super().__init__(simulate=simulate)
        self.pipeline = rs.pipeline()
        self.pipeline.start()

        self.depth_image = np.array([[]])
        self.color_image = np.array([[]])


    def get_rgb_depth(self, depth_for_display=False):
        frames = self.pipeline.wait_for_frames()
        depth = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        
        if depth and color_frame:
            self.depth_image = np.asanyarray(depth.get_data())
            self.color_image = np.asanyarray(color_frame.get_data())
        return self.color_image, self.depth_image
            
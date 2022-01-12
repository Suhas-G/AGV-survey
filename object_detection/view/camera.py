from abc import ABC, abstractmethod
from typing import Tuple

# import pyzed.sl as sl
import numpy as np
# import numpy.typing as npt

import pyrealsense2 as rs



class Camera(ABC):
    def __init__(self, simulate = False) -> None:
        super().__init__()
        self.simulate = simulate

    @abstractmethod
    def get_rgb_depth(self):
        pass



# class ZEDCamera(Camera):
#     def __init__(self, simulate=False) -> None:
#         super().__init__(simulate=simulate)
#         self.zed = sl.Camera()
#         # Create a InitParameters object and set configuration parameters
#         init_params = sl.InitParameters()
#         init_params.camera_resolution = sl.RESOLUTION.HD1080  # Use HD1080 video mode
#         init_params.camera_fps = 30  # Set fps at 30

#         # Open the camera
#         err = self.zed.open(init_params)
#         if err != sl.ERROR_CODE.SUCCESS:
#             exit(1)

#         print('ZED Camera opened')
#         camera_resolution = self.zed.get_camera_information().camera_resolution
#         self.image_zed = sl.Mat(camera_resolution.width, camera_resolution.height, sl.MAT_TYPE.U8_C3)
#         self.depth_zed = sl.Mat(camera_resolution.width, camera_resolution.height)
#         self.runtime_parameters = sl.RuntimeParameters()

#     def get_rgb_depth(self, depth_for_display = False) -> Tuple[npt.NDArray, npt.NDArray]:
#         if self.zed.grab(self.runtime_parameters) == sl.ERROR_CODE.SUCCESS:
#             self.zed.retrieve_image(self.image_zed, sl.VIEW.LEFT)
#             if depth_for_display:
#                 self.zed.retrieve_image(self.depth_zed, sl.VIEW.DEPTH)
#             else:
#                 self.zed.retrieve_measure(self.depth_zed, sl.MEASURE.DEPTH)

#         return self.image_zed.get_data()[:, :, :-1], self.depth_zed.get_data()




class RealSenseCamera(Camera):
    def __init__(self, simulate=False) -> None:
        super().__init__(simulate=simulate)
        self.pipeline = rs.pipeline()
        cfg = rs.config()
        
        if simulate:
            cfg.enable_device_from_file("shoes5.bag")
        self.profile = self.pipeline.start(cfg)
       
        self.align = rs.align(rs.stream.color)
        depth_profile = self.profile.get_stream(rs.stream.depth) # Fetch stream profile for depth stream
        self.intrinsic = depth_profile.as_video_stream_profile().get_intrinsics()
        self.point_cloud = rs.pointcloud()
        self.colorizer = rs.colorizer()
        self.depth_image = np.array([[]])
        self.color_image = np.array([[]])


    def get_rgb_depth(self, depth_for_display=False):
        frames = self.pipeline.wait_for_frames()
        frames = self.align.process(frames)
        self.depth = frames.get_depth_frame()
        # depth.get_intrinsics()
        self.color_frame = frames.get_color_frame()
        
        if self.depth and self.color_frame:
            if depth_for_display:
                self.depth_image = np.asanyarray(self.colorizer.colorize(self.depth).get_data())
            else:
                self.depth_image = np.asanyarray(self.depth.get_data()).astype(float)
                depth_scale = self.profile.get_device().first_depth_sensor().get_depth_scale()
                self.depth_image = self.depth_image * depth_scale
            self.color_image = np.asanyarray(self.color_frame.get_data())
        return self.color_image, self.depth_image


    def get_3d_coordinates(self, bbox, center):
        points = self.point_cloud.calculate(self.depth)
        verts = np.asanyarray(points.get_vertices()).view(np.float32).reshape(-1, self.intrinsic.width, 3)
        obj_points = verts[int(bbox[1]):int(bbox[1] + bbox[3]), int(bbox[0]):int(bbox[0] + bbox[2])]
        zs = obj_points[:, :, 2]
        xs = verts[:, :, 0]
        ys = verts[:, :, 1]
        z = np.median(zs)
        return xs[center[1], center[0]], ys[center[1], center[0]],  z
            
import numpy as np
import pyrealsense2 as rs




class RealSenseCamera:
    def __init__(self, simulate=False, filepath = None) -> None:
        super().__init__()
        self.pipeline = rs.pipeline()
        cfg = rs.config()
        
        if simulate:
            cfg.enable_device_from_file(filepath)
        self.profile = self.pipeline.start(cfg)
       
        self.align = rs.align(rs.stream.color)
        # depth_profile = self.profile.get_stream(rs.stream.depth) # Fetch stream profile for depth stream
        # self.intrinsic = depth_profile.as_video_stream_profile().get_intrinsics()
        rgb_profile = self.profile.get_stream(rs.stream.color) # Fetch stream profile for color stream
        self.intrinsic = rgb_profile.as_video_stream_profile().get_intrinsics()
        self.point_cloud = rs.pointcloud()
        self.colorizer = rs.colorizer()
        self.depth_image = np.array([[]])
        self.color_image = np.array([[]])


    def get_rgb_depth(self, depth_for_display=False):
        frames = self.pipeline.wait_for_frames()
        frames = self.align.process(frames)
        self.depth = frames.get_depth_frame()
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
        # return xs[center[0], center[1]], ys[center[0], center[1]],  z
            
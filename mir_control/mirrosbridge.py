# -*- coding: utf-8 -*-
import roslibpy
import time
import threading
import requests
import re

from .config import MIR_WEB_SESSION_USERNAME, MIR_WEB_SESSION_PASSWORD


class MirRosBridge:
    def __init__(self, host, port):
        self.data = {
            'velocity': [],
            'pose': [],
            'odometry': [],
            'amcl': [],
            'encoders': [],
            'imu': [],
            'goal': [],
            'joystick': []
        }
        self.client = roslibpy.Ros(host, port)
        self.client.run()
        print("Is ROS connected?", self.client.is_connected)

        roslibpy.Topic(self.client, "/cmd_vel", "geometry_msgs/TwistStamped").subscribe(self.cb_cmd_vel)
        roslibpy.Topic(self.client, "/robot_pose", "geometry_msgs/Pose").subscribe(self.cb_pose)
        roslibpy.Topic(self.client, "/odom", "nav_msgs/Odometry").subscribe(self.cb_odometry)
        roslibpy.Topic(self.client, "/amcl_pose", "geometry_msgs/PoseWithCovarianceStamped").subscribe(self.cb_amcl_pose)
        roslibpy.Topic(self.client, "/MC/encoders", "sdc21x0/StampedEncoders").subscribe(self.cb_encoders)
        roslibpy.Topic(self.client, "/imu_data", "sensor_msgs/Imu").subscribe(self.cb_imu)
        roslibpy.Topic(self.client, "/move_base/goal", "mir_actions/MirMoveBaseActionGoal").subscribe(self.cb_goal)
        roslibpy.Topic(self.client, "/joystick_vel", "mir_msgs/JoystickVel").subscribe(self.cb_joystick)

        self.talker = roslibpy.Topic(
            self.client, "/joystick_vel", "mir_msgs/JoystickVel"
        )
        self.web_session_id = ""
        self.get_web_session_id(url="http://"+host+"/?mode=log-in")
        self.token = ""
        self.token_thread = threading.Thread(
            target=self.keep_joystick_token_alive, args=()
        )

    def cb_cmd_vel(self, msg):
        self.data['velocity'] = msg

    def cb_pose(self, msg):
        self.data['pose'] = msg

    def cb_odometry(self, msg):
        self.data['odometry'] = msg

    def cb_amcl_pose(self, msg):
        self.data['amcl'] = msg

    def cb_encoders(self, msg):
        self.data['encoders'] = msg

    def cb_imu(self, msg):
        self.data['imu'] = msg

    def cb_goal(self, msg):
        self.data['goal'] = msg

    def cb_joystick(self, msg):
        self.data['joystick'] = msg

    def get_web_session_id(self, url):
        body = {"login_username": MIR_WEB_SESSION_USERNAME, "login_password": MIR_WEB_SESSION_PASSWORD}
        x = requests.post(url, data=body)
        # web session id has a length of 26 and consists of numbers and lowercase letters
        web_session_string = re.findall(
            "PHPSESSID=[a-z0-9]{26}", x.headers["Set-Cookie"]
        )[0]
        self.web_session_id = web_session_string[-26:]

    def set_manual_mode(self):
        service = roslibpy.Service(
            self.client, "/mirsupervisor/setRobotState", "mirSupervisor/SetState"
        )
        request = roslibpy.ServiceRequest(
            dict(robotState=11, web_session_id=self.web_session_id)
        )
        result = service.call(request)
        # Get token necessary for manual operation
        self.token = result["joystick_token"]

    def set_continous_manual_control(self, state):
        if state:
            self.set_manual_mode()
            self.token_thread.start()
        else:
            self.stop_continous_manual_mode()

    def keep_joystick_token_alive(self):
        try:
            while self.client.is_connected:
                self.talker.publish(
                    roslibpy.Message(
                        {
                            "joystick_token": self.token,
                            "speed_command": {
                                "linear": {"x": 0.0, "y": 0, "z": 0},
                                "angular": {"x": 0, "y": 0, "z": 0},
                            },
                        }
                    )
                )
                time.sleep(5)
        except (
            RuntimeError,
            TypeError,
            NameError,
            KeyboardInterrupt,
            AttributeError,
        ) as e:
            print(e)
            self.client.terminate()
            self.token_thread.join()

    def move(self, linear, angular):
        try:
            self.talker.publish(
                roslibpy.Message(
                    {
                        "joystick_token": self.token,
                        "speed_command": {
                            "linear": {"x": linear[0], "y": linear[1], "z": linear[2]},
                            "angular": {"x": angular[0], "y": angular[1], "z": angular[2]},
                        },
                    }
                )
            )
        except:
            raise ValueError("Lost connection to MiR in move()")

    @staticmethod
    def turn_left():
        linear_velocity = [0.0, 0.0, 0.0]
        angular_velocity = [0.0, 0.0, 0.4]
        return [linear_velocity, angular_velocity]

    @staticmethod
    def turn_right():
        linear_velocity = [0.0, 0.0, 0.0]
        angular_velocity = [0.0, 0.0, -0.4]
        return [linear_velocity, angular_velocity]

    @staticmethod
    def forward():
        linear_velocity = [0.2, 0.0, 0.0]
        angular_velocity = [0.0, 0.0, 0.0]
        return [linear_velocity, angular_velocity]

    @staticmethod
    def backward():
        linear_velocity = [-0.2, 0.0, 0.0]
        angular_velocity = [0.0, 0.0, 0.0]
        return [linear_velocity, angular_velocity]

    def stop_continous_manual_mode(self):
        self.token_thread.join()

    def close_bridge(self):
        self.client.terminate()

    def shutdown(self):
        self.stop_continous_manual_mode()
        self.close_bridge()

    def subscribe_to_topic(self, topic, message_type, callback, **kwargs):
        roslibpy.Topic(self.client, topic, message_type, **kwargs).subscribe(callback)

    def get_publisher_to_topic(self, topic, message_type, **kwargs):
        return roslibpy.Topic(self.client, topic, message_type, **kwargs)

    def get_topics(self):
        return self.client.get_topics()

    def get_services(self):
        return self.client.get_services()
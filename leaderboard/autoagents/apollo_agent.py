#!/usr/bin/env python

# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
This module provides an NPC agent to control the ego vehicle
"""

from __future__ import print_function

import carla
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider

from leaderboard.autoagents.autonomous_agent import AutonomousAgent, Track
import subprocess
import threading
import time


def get_entry_point():
    return "ApolloAgent"


class ApolloAgent(AutonomousAgent):
    """
    ApolloAgent
    """

    _agent = None
    _route_assigned = False
    _apollo_config = None
    _apollo_config_loader = None
    x = 0
    y = 0
    z = 0
    yaw = 0
    m = "Town04"

    def _get_apollo_config(self):
        """Get Apollo configuration, loading it on first access."""
        if self._apollo_config is None:
            try:
                # Import here to avoid circular imports and path issues
                import sys
                from pathlib import Path
                
                # Add src directory to path for imports
                current_file = Path(__file__).resolve()
                # Navigate to project root (leaderboard -> dependencies -> Carlo -> src)
                project_root = current_file.parent.parent.parent.parent.parent
                src_path = project_root / "src"
                if str(src_path) not in sys.path:
                    sys.path.insert(0, str(src_path))
                
                from utils.apollo_config_loader import get_apollo_config_loader
                loader = get_apollo_config_loader()
                container_name = loader.get_container_name()
                user_name = loader.get_user_name()
                self._apollo_config = {
                    'container_name': container_name,
                    'user_name': user_name
                }
                print(f"Apollo config loaded: container={container_name}, user={user_name}")
            except Exception as e:
                print(f"Failed to load Apollo configuration: {e}")
                print("Using default Apollo settings")
                self._apollo_config = {
                    'container_name': 'apollo_dev_tay',
                    'user_name': 'tay'
                }
                
        return self._apollo_config

    def _get_container_name(self) -> str:
        """Get Apollo container name from config."""
        config = self._get_apollo_config()
        return config['container_name']

    def _get_container_user(self) -> str:
        """Get Apollo container user from config."""
        config = self._get_apollo_config()
        return config['user_name']

    def _get_workdir(self) -> str:
        """Get Apollo container working directory."""
        return "/apollo/modules/apollo-bridge"

    def _get_start_script(self) -> str:
        """Get Apollo start script name."""
        return "start.sh"

    def _get_kill_script(self) -> str:
        """Get Apollo kill script name."""
        return "kill.sh"

    def setup(self, path_to_conf_file):
        """
        Setup the agent parameters
        """
        self.track = Track.SENSORS

        self._agent = None

        # Find destination

        # Execute the Apollo
        start_script = self._get_start_script()
        command = f"/bin/bash {start_script} -x {self.x} -y {self.y} -z {self.z} -yaw {self.yaw} -m {self.m}"
        docker_exec_command = f"docker exec --user {self._get_container_user()} -w {self._get_workdir()} {self._get_container_name()} {command}"
        print(f"Executing Apollo start command: {command}")
        print(f"Docker exec command: {docker_exec_command}")
        process = subprocess.Popen(
            docker_exec_command,
            shell=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )

    def sensors(self):
        """
        Define the sensor suite required by the agent

        :return: a list containing the required sensors in the following format:

        [
            {'type': 'sensor.camera.rgb', 'x': 0.7, 'y': -0.4, 'z': 1.60, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
                      'width': 300, 'height': 200, 'fov': 100, 'id': 'Left'},

            {'type': 'sensor.camera.rgb', 'x': 0.7, 'y': 0.4, 'z': 1.60, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
                      'width': 300, 'height': 200, 'fov': 100, 'id': 'Right'},

            {'type': 'sensor.lidar.ray_cast', 'x': 0.7, 'y': 0.0, 'z': 1.60, 'yaw': 0.0, 'pitch': 0.0, 'roll': 0.0,
             'id': 'LIDAR'}
        ]
        """

        # id will be set to role_name
        sensors = [
            {
                "type": "sensor.other.gnss",
                "x": 1,
                "y": 0,
                "z": 2,
                "roll": 0.0,
                "pitch": 0.0,
                "yaw": 0.0,
                "noise_alt_stddev": 0.0,
                "noise_lat_stddev": 0.0,
                "noise_lon_stddev": 0.0,
                "noise_alt_bias": 0.0,
                "noise_lat_bias": 0.0,
                "noise_lon_bias": 0.0,
                "id": "gnss-osg",
            },
            {
                "type": "sensor.other.imu",
                "x": 2,
                "y": 0,
                "z": 2,
                "roll": 0.0,
                "pitch": 0.0,
                "yaw": 0.0,
                "noise_accel_stddev_x": 0.0,
                "noise_accel_stddev_y": 0.0,
                "noise_accel_stddev_z": 0.0,
                "noise_gyro_stddev_x": 0.0,
                "noise_gyro_stddev_y": 0.0,
                "noise_gyro_stddev_z": 0.0,
                "noise_gyro_bias_x": 0.0,
                "noise_gyro_bias_y": 0.0,
                "noise_gyro_bias_z": 0.0,
                "id": "imu-osg",
            },
            {
                "type": "sensor.lidar.ray_cast",
                "x": 0.0,
                "y": 0.0,
                "z": 2.40,
                "roll": 0.0,
                "pitch": 0.0,
                "yaw": 0.0,
                "range": 50,
                "channels": 128,
                "points_per_second": 320000,
                "upper_fov": 2.0,
                "lower_fov": -26.8,
                "rotation_frequency": 20,
                "noise_stddev": 0.0,
                "id": "ray_cast-osg",
            },
            {
                "type": "sensor.camera.rgb",
                "x": 2.5,
                "y": 0,
                "z": 1.2,
                "roll": 0.0,
                "pitch": 0.0,
                "yaw": 0.0,
                "width": 800,
                "height": 600,
                "fov": 90,
                "fstop": 8.0,
                "id": "rgb-6-osg",
            },
        ]

        return sensors

    def run_step(self, input_data, timestamp):
        """
        Execute one step of navigation.
        """

        return -1

    def __call__(self):
        """
        Execute the agent call, e.g. agent()
        Returns the next vehicle controls
        """

        return -1

    def get_control(self):
        return self._agent.get_control()

    def destroy(self):
        """
        Destroy (clean-up) the agent
        :return:
        """
        print("Run destroy")
        kill_script = self._get_kill_script()
        command = f"/bin/bash {kill_script}"
        docker_exec_command = f"docker exec --user {self._get_container_user()} -w {self._get_workdir()} {self._get_container_name()} {command}"
        print(f"Executing Apollo kill command: {command}")
        print(f"Docker exec command: {docker_exec_command}")
        process = subprocess.Popen(
            docker_exec_command,
            shell=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        time.sleep(3)

    def set_global_plan(self, global_plan_gps, global_plan_world_coord):
        """
        Set the plan (route) for the agent
        """
        # Set the destination and map name
        dest_tf = global_plan_world_coord[-1][0]
        carla_map = CarlaDataProvider.get_map()
        wp = carla_map.get_waypoint(dest_tf.location)
        wp = wp.next(10)[0]
        dest_lc = wp.transform.location
        dest_yaw = wp.transform.rotation.yaw
        self.x = dest_lc.x
        self.y = dest_lc.y
        self.z = dest_lc.z
        self.yaw = dest_yaw
        self.m = carla_map.name.split("/")[-1]

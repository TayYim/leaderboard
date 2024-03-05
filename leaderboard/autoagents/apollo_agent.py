#!/usr/bin/env python

# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
This module provides an NPC agent to control the ego vehicle
"""

from __future__ import print_function

import carla
from agents.navigation.basic_agent import BasicAgent
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider

from leaderboard.autoagents.autonomous_agent import AutonomousAgent, Track


def get_entry_point():
    return "ApolloAgent"


class ApolloAgent(AutonomousAgent):
    """
    ApolloAgent
    """

    _agent = None
    _route_assigned = False

    def setup(self, path_to_conf_file):
        """
        Setup the agent parameters
        """
        self.track = Track.SENSORS

        self._agent = None

        self._ego_vehicle = None

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
        # if not self._ego_vehicle:

        #     # Search for the ego actor
        #     hero_actor = None
        #     for actor in CarlaDataProvider.get_world().get_actors():
        #         if (
        #             "role_name" in actor.attributes
        #             and actor.attributes["role_name"] == "hero"
        #         ):
        #             hero_actor = actor
        #             self._ego_vehicle = hero_actor
        #             break

        #     if not hero_actor:
        #         return carla.VehicleControl()

        #     return carla.VehicleControl()

        # else:
        #     return -1
        return -1
    
    def __call__(self):
        """
        Execute the agent call, e.g. agent()
        Returns the next vehicle controls
        """

        return -1

    def get_control(self):
        return self._agent.get_control()

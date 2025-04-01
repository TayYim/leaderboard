#!/usr/bin/env python

# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
This module provides an NPC agent to control the ego vehicle
"""

from __future__ import print_function

import carla
from adaptive_behavior_agent import AdaptiveBehaviorAgent
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider

from leaderboard.autoagents.autonomous_agent import AutonomousAgent, Track

def get_entry_point():
    return 'AbaAgent'

class AbaAgent(AutonomousAgent):

    """
    Adaptive BA autonomous agent to control the ego vehicle
    """

    _agent = None
    _route_assigned = False

    def setup(self, path_to_conf_file):
        """
        Setup the agent parameters
        """
        self.track = Track.SENSORS

        self._agent = None

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

        sensors = [
            {'type': 'sensor.camera.rgb', 'x': 0.7, 'y': -0.4, 'z': 1.60, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
             'width': 300, 'height': 200, 'fov': 100, 'id': 'Left'},
        ]

        return sensors

    def run_step(self, input_data, timestamp):
        """
        Execute one step of navigation. 
        """
        if not self._agent:

            # Search for the ego actor
            hero_actor = None
            for actor in CarlaDataProvider.get_world().get_actors():
                if 'role_name' in actor.attributes and actor.attributes['role_name'] == 'hero':
                    hero_actor = actor
                    break

            if not hero_actor:
                return carla.VehicleControl()

            # Add an agent that follows the route to the ego
            # self._agent = BehaviorAgent(hero_actor)
            self._agent = AdaptiveBehaviorAgent(hero_actor, debug_trajectory=True)

            plan = []
            prev_wp = None
            for transform, _ in self._global_plan_world_coord:
                wp = CarlaDataProvider.get_map().get_waypoint(transform.location)
                if prev_wp:
                    plan.extend(self._agent.trace_route(prev_wp, wp))
                prev_wp = wp

            self._agent.set_global_plan(plan)
            
            # Visualize the planned trajectory
            # self._draw_trajectory(plan)

            return carla.VehicleControl()

        else:
            return self._agent.run_step()
            
    def _draw_trajectory(self, plan):
        """
        Draw the planned trajectory in the simulator
        
        Args:
            plan: List of waypoints that define the trajectory
        """
        if not plan:
            return
            
        debug_helper = CarlaDataProvider.get_world().debug
        
        # Use a bright color that's easily visible (red)
        color = carla.Color(r=255, g=0, b=0, a=255)
        
        # Draw lines between consecutive waypoints
        # Setting lifetime to 0 makes the lines persist indefinitely
        for i in range(len(plan) - 1):
            start_location = plan[i][0].transform.location
            start_location.z += 0.5  # Lift the line slightly above the road
            
            end_location = plan[i+1][0].transform.location
            end_location.z += 0.5  # Lift the line slightly above the road
            
            # Draw a line connecting the waypoints
            debug_helper.draw_line(
                start_location,
                end_location,
                thickness=0.2,
                color=color,
                life_time=0.0  # Persist indefinitely
            )

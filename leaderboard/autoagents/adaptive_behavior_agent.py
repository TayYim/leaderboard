# Copyright (c) 2018-2020 CVC.
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
This module extends the BehaviorAgent to include overtaking capabilities
for slow vehicles ahead.
"""

import carla
import numpy as np
import math
from agents.navigation.behavior_agent import BehaviorAgent
from agents.navigation.local_planner import RoadOption
from agents.tools.misc import get_speed

class AdaptiveBehaviorAgent(BehaviorAgent):
    """
    AdaptiveBehaviorAgent extends BehaviorAgent by adding overtaking capabilities.
    The agent will check for slow vehicles ahead and try to overtake them when safe.
    """

    def __init__(self, vehicle, behavior='normal', opt_dict={}, map_inst=None, grp_inst=None, debug_trajectory=False):
        """
        Constructor method.

            :param vehicle: actor to apply to local planner logic onto
            :param behavior: type of agent to apply
            :param debug_trajectory: whether to visualize the trajectory in the CARLA world
        """
        super().__init__(vehicle, behavior, opt_dict, map_inst, grp_inst)
        
        # Parameters for overtaking behavior
        self._min_speed_advantage = 2  # Minimum speed difference to consider overtaking (m/s)
        self._overtake_min_speed = 3  # Minimum speed required to consider overtaking (m/s)
        self._overtake_distance_threshold = 30  # Distance threshold to trigger overtaking
        
        # Parameters for lane change safety
        self._min_lane_change_distance = 7.0  # Minimum distance to vehicles in target lane (lateral)
        self._side_distance_threshold = 15.0  # Maximum distance to check for vehicles on the sides
        self._side_detection_angle = 90  # Angle in degrees to check for vehicles on the sides
        
        # Visualization parameters
        self._debug_trajectory = debug_trajectory
        self._world = self._vehicle.get_world()
        self._debug = self._world.debug
        self._visualization_time = 0.1  # How long each visualization stays visible (seconds)
        self._trajectory_points = []  # Store visualization objects to clean up
        
        # Color presets for visualization
        self._regular_route_color = carla.Color(0, 255, 0)  # Green for regular route
        self._overtaking_color = carla.Color(255, 165, 0)  # Orange for overtaking
        self._target_point_color = carla.Color(255, 0, 0)  # Red for target points
        self._point_size = 0.1
        self._line_thickness = 0.1
        self._height_offset = 0.5  # Draw the trajectory slightly above the ground

    def _check_overtake(self, waypoint, vehicle_list):
        """
        This method checks if there's a slow vehicle ahead that
        should be overtaken.

            :param waypoint: current waypoint of the agent
            :param vehicle_list: list of all the nearby vehicles
        """
        # Don't overtake if current speed is too low
        if self._speed < self._overtake_min_speed:
            return

        # Check for vehicles ahead
        vehicle_state, vehicle, distance = self._vehicle_obstacle_detected(
            vehicle_list, self._overtake_distance_threshold, up_angle_th=30)

        if not vehicle_state:
            return

        # Calculate actual distance between vehicles
        distance = distance - max(
            vehicle.bounding_box.extent.y, vehicle.bounding_box.extent.x) - max(
                self._vehicle.bounding_box.extent.y, self._vehicle.bounding_box.extent.x)
        
        # Check if the vehicle ahead is slower than us
        vehicle_speed = get_speed(vehicle)
        
        # Only overtake if the vehicle ahead is significantly slower
        if self._speed - vehicle_speed < self._min_speed_advantage:
            return
        
        left_turn = waypoint.left_lane_marking.lane_change
        right_turn = waypoint.right_lane_marking.lane_change
        
        left_wpt = waypoint.get_left_lane()
        right_wpt = waypoint.get_right_lane()
        
        # First try to overtake from the left (standard practice)
        if (left_turn == carla.LaneChange.Left or left_turn == carla.LaneChange.Both) and \
                waypoint.lane_id * left_wpt.lane_id > 0 and left_wpt.lane_type == carla.LaneType.Driving:
            
            # Check for vehicles in the left lane (both ahead and beside)
            if self._is_lane_safe_for_change(vehicle_list, -1):
                print("Slow vehicle ahead! Overtaking from the left")
                self._execute_lane_change('left', waypoint, left_wpt)
                return

        # If left lane is not available or occupied, try right lane
        elif (right_turn == carla.LaneChange.Right or right_turn == carla.LaneChange.Both) and \
                waypoint.lane_id * right_wpt.lane_id > 0 and right_wpt.lane_type == carla.LaneType.Driving:
            
            # Check for vehicles in the right lane (both ahead and beside)
            if self._is_lane_safe_for_change(vehicle_list, 1):
                print("Slow vehicle ahead! Overtaking from the right")
                self._execute_lane_change('right', waypoint, right_wpt)

    def _is_lane_safe_for_change(self, vehicle_list, lane_offset):
        """
        Thoroughly checks if the target lane is safe for a lane change.
        Looks for vehicles both ahead and directly beside the agent.
        
        Args:
            vehicle_list: List of vehicles to check against
            lane_offset: -1 for left lane, 1 for right lane
            
        Returns:
            bool: True if the lane is safe for changing
        """
        # First check for vehicles ahead in the target lane
        ahead_vehicle_state, _, _ = self._vehicle_obstacle_detected(
            vehicle_list, self._overtake_distance_threshold, up_angle_th=60, lane_offset=lane_offset)
        
        # If there's a vehicle ahead in target lane, it's not safe
        if ahead_vehicle_state:
            return False
            
        # Now check for vehicles right beside us in the target lane
        ego_location = self._vehicle.get_location()
        ego_transform = self._vehicle.get_transform()
        ego_forward = ego_transform.get_forward_vector()
        ego_right = ego_transform.get_right_vector()
        
        # Scale the right vector based on the lane offset
        side_check_vector = carla.Vector3D(
            ego_right.x * lane_offset,
            ego_right.y * lane_offset,
            ego_right.z * lane_offset
        )
        
        # Check each vehicle to see if it's beside us in the target lane
        for vehicle in vehicle_list:
            if vehicle.id == self._vehicle.id:
                continue
                
            vehicle_location = vehicle.get_location()
            
            # Vector from our vehicle to the other vehicle
            to_vehicle = vehicle_location - ego_location
            
            # Distance to the other vehicle
            distance = ego_location.distance(vehicle_location)
            
            # We're not interested in vehicles that are too far away
            if distance > self._side_distance_threshold:
                continue
                
            # Calculate dot product with our forward vector to see if it's beside/ahead/behind
            forward_dot = ego_forward.x * to_vehicle.x + ego_forward.y * to_vehicle.y
            
            # Calculate dot product with our side check vector to see if it's in the target lane
            side_dot = side_check_vector.x * to_vehicle.x + side_check_vector.y * to_vehicle.y
            
            # If the vehicle is beside us (small forward_dot) and in the target lane (positive side_dot)
            # then the lane is not safe
            if abs(forward_dot) < self._min_lane_change_distance and side_dot > 0:
                print(f"Vehicle detected beside us in target lane at distance {distance:.2f}m. Aborting lane change.")
                return False
                
        # If we passed all checks, the lane is safe
        return True

    def _execute_lane_change(self, direction, current_waypoint, target_lane_waypoint):
        """
        Execute a lane change maneuver by creating a new temporary route plan
        
        Args:
            direction: 'left' or 'right'
            current_waypoint: Current waypoint of the agent
            target_lane_waypoint: Waypoint in the target lane
        """
        # Create waypoints for the lane change maneuver
        route_plan = []
        
        # First add the immediate lane change waypoint
        next_wp = target_lane_waypoint
        route_plan.append((next_wp, RoadOption.CHANGELANERIGHT if direction == 'right' else RoadOption.CHANGELANELEFT))
        
        # Then continue in the new lane for a short distance to ensure smooth transition
        for _ in range(5):
            next_wp = next_wp.next(2.0)[0]
            route_plan.append((next_wp, RoadOption.LANEFOLLOW))
        
        # Create a temporary route plan for the lane change
        self._local_planner.set_global_plan(route_plan, stop_waypoint_creation=False)
        
        # Make lane change more aggressive based on behavior
        if self._behavior.__class__.__name__ == "Aggressive":
            self._local_planner._target_speed = min(self._speed_limit * 1.1, self._behavior.max_speed)
        else:
            self._local_planner._target_speed = min(self._speed_limit, self._behavior.max_speed)
            
        # Visualize the overtaking path
        if self._debug_trajectory:
            self._draw_overtake_trajectory(route_plan)

    def _draw_plan_trajectory(self):
        """
        Visualizes the current planned trajectory in the CARLA world.
        """
        self._clear_trajectory_visualization()
        
        if not self._local_planner._waypoints_queue:
            return
            
        # Draw the entire waypoint queue
        prev_loc = self._vehicle.get_location()
        prev_loc.z += self._height_offset  # Lift above ground
        
        for i, (waypoint, _) in enumerate(self._local_planner._waypoints_queue):
            loc = waypoint.transform.location
            loc.z += self._height_offset  # Lift above ground
            
            # Draw a line from previous point to this point
            line = self._debug.draw_line(
                prev_loc,
                loc,
                thickness=self._line_thickness,
                color=self._regular_route_color,
                life_time=self._visualization_time
            )
            self._trajectory_points.append(line)
            
            # Draw a point at each waypoint
            point = self._debug.draw_point(
                loc,
                size=self._point_size,
                color=self._regular_route_color,
                life_time=self._visualization_time
            )
            self._trajectory_points.append(point)
            
            prev_loc = loc
            
        # Try to highlight next target waypoint
        try:
            if hasattr(self._local_planner, '_waypoint_buffer') and self._local_planner._waypoint_buffer:
                target_loc = self._local_planner._waypoint_buffer[0][0].transform.location
                target_loc.z += self._height_offset
                target = self._debug.draw_point(
                    target_loc,
                    size=self._point_size*2,
                    color=self._target_point_color,
                    life_time=self._visualization_time
                )
                self._trajectory_points.append(target)
            elif hasattr(self._local_planner, '_target_waypoint') and self._local_planner._target_waypoint:
                target_loc = self._local_planner._target_waypoint.transform.location
                target_loc.z += self._height_offset
                target = self._debug.draw_point(
                    target_loc,
                    size=self._point_size*2,
                    color=self._target_point_color,
                    life_time=self._visualization_time
                )
                self._trajectory_points.append(target)
            elif self._local_planner._waypoints_queue:
                target_loc = self._local_planner._waypoints_queue[0][0].transform.location
                target_loc.z += self._height_offset
                target = self._debug.draw_point(
                    target_loc,
                    size=self._point_size*2,
                    color=self._target_point_color,
                    life_time=self._visualization_time
                )
                self._trajectory_points.append(target)
        except Exception as e:
            print(f"Warning: Could not highlight target waypoint: {e}")

    def _draw_overtake_trajectory(self, route_plan):
        """
        Visualizes the overtaking trajectory.
        
        Args:
            route_plan: List of (waypoint, road_option) tuples for the overtaking path
        """
        self._clear_trajectory_visualization()
        
        if not route_plan:
            return
            
        # Draw the overtaking path
        prev_loc = self._vehicle.get_location()
        prev_loc.z += self._height_offset
        
        for i, (waypoint, _) in enumerate(route_plan):
            loc = waypoint.transform.location
            loc.z += self._height_offset
            
            line = self._debug.draw_line(
                prev_loc,
                loc,
                thickness=self._line_thickness,
                color=self._overtaking_color,
                life_time=self._visualization_time
            )
            self._trajectory_points.append(line)
            
            point = self._debug.draw_point(
                loc,
                size=self._point_size,
                color=self._overtaking_color,
                life_time=self._visualization_time
            )
            self._trajectory_points.append(point)
            
            prev_loc = loc

    def _clear_trajectory_visualization(self):
        """
        Clears all trajectory visualization objects.
        """
        self._trajectory_points = []

    def run_step(self, debug=False):
        """
        Execute one step of navigation.
        Overrides the parent class to add overtaking behavior.

            :param debug: boolean for debugging
            :return control: carla.VehicleControl
        """
        self._update_information()

        control = None
        if self._behavior.tailgate_counter > 0:
            self._behavior.tailgate_counter -= 1

        ego_vehicle_loc = self._vehicle.get_location()
        ego_vehicle_wp = self._map.get_waypoint(ego_vehicle_loc)

        # 1: Red lights and stops behavior
        if self.traffic_light_manager():
            return self.emergency_stop()

        # 2.1: Pedestrian avoidance behaviors
        walker_state, walker, w_distance = self.pedestrian_avoid_manager(ego_vehicle_wp)

        if walker_state:
            # Distance is computed from the center of the two cars,
            # we use bounding boxes to calculate the actual distance
            distance = w_distance - max(
                walker.bounding_box.extent.y, walker.bounding_box.extent.x) - max(
                    self._vehicle.bounding_box.extent.y, self._vehicle.bounding_box.extent.x)

            # Emergency brake if the car is very close.
            if distance < self._behavior.braking_distance:
                return self.emergency_stop()

        # 2.2: Car following behaviors
        vehicle_list = self._world.get_actors().filter("*vehicle*")
        def dist(v): return v.get_location().distance(ego_vehicle_wp.transform.location)
        vehicle_list = [v for v in vehicle_list if dist(v) < 45 and v.id != self._vehicle.id]

        # Check for overtaking opportunities
        self._check_overtake(ego_vehicle_wp, vehicle_list)

        # Continue with normal car following behavior
        vehicle_state, vehicle, distance = self.collision_and_car_avoid_manager(ego_vehicle_wp)

        if vehicle_state:
            # Distance is computed from the center of the two cars,
            # we use bounding boxes to calculate the actual distance
            distance = distance - max(
                vehicle.bounding_box.extent.y, vehicle.bounding_box.extent.x) - max(
                    self._vehicle.bounding_box.extent.y, self._vehicle.bounding_box.extent.x)

            # Emergency brake if the car is very close.
            if distance < self._behavior.braking_distance:
                return self.emergency_stop()
            else:
                control = self.car_following_manager(vehicle, distance)

        # 3: Intersection behavior
        if not control:
            if self._incoming_waypoint.is_junction and (self._incoming_direction in [RoadOption.LEFT, RoadOption.RIGHT]):
                target_speed = min([
                    self._behavior.max_speed,
                    self._speed_limit - 5])
                self._local_planner.set_speed(target_speed)
                control = self._local_planner.run_step(debug=debug)
            else:
                # 4: Normal behavior
                target_speed = min([
                    self._behavior.max_speed,
                    self._speed_limit - self._behavior.speed_lim_dist])
                self._local_planner.set_speed(target_speed)
                control = self._local_planner.run_step(debug=debug)

        # If debug trajectory is enabled, update the visualization
        if self._debug_trajectory:
            self._draw_plan_trajectory()

        return control

    def set_debug_trajectory(self, enable=True):
        """
        Enable or disable trajectory visualization.
        
        Args:
            enable: Whether to enable trajectory visualization
        """
        self._debug_trajectory = enable
        
        if enable:
            self._draw_plan_trajectory()
        else:
            self._clear_trajectory_visualization()

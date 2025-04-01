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
        self._overtake_counter = 0  # Counter to prevent constant lane changes
        self._min_speed_advantage = 2  # Minimum speed difference to consider overtaking (m/s)
        self._overtake_min_speed = 3  # Minimum speed required to consider overtaking (m/s)
        self._overtake_distance_threshold = 30  # Distance threshold to trigger overtaking
        self._overtake_time = 2  # Time to wait before attempting another overtake
        
        # Overtaking state variables
        self._overtaking = False
        self._overtake_direction = None  # 'left' or 'right'
        self._original_waypoint = None
        self._target_lane_id = None
        self._return_point = None
        self._lane_change_distance = 50  # Distance to travel in the new lane before returning
        self._original_route = None  # Store the complete original route
        
        # New variables to track overtaking progress
        self._vehicle_being_overtaken = None  # The vehicle we're currently overtaking
        self._min_distance_passed = 20  # Minimum distance to be ahead of overtaken vehicle
        self._overtake_completion_distance = 30  # Distance to travel in new lane after passing vehicle
        self._return_to_original_route = True  # Flag to determine if we should return to original route

        # Visualization parameters
        self._debug_trajectory = debug_trajectory
        self._world = self._vehicle.get_world()
        self._debug = self._world.debug
        self._visualization_time = 0.1  # How long each visualization stays visible (seconds)
        self._trajectory_points = []  # Store visualization objects to clean up
        
        # Color presets for visualization
        self._regular_route_color = carla.Color(0, 255, 0)  # Green for regular route
        self._overtaking_color = carla.Color(255, 165, 0)  # Orange for overtaking
        self._connection_path_color = carla.Color(0, 0, 255)  # Blue for connection paths
        self._target_point_color = carla.Color(255, 0, 0)  # Red for target points
        self._point_size = 0.1
        self._line_thickness = 0.1
        self._height_offset = 0.5  # Draw the trajectory slightly above the ground

    def set_global_plan(self, plan, stop_waypoint_creation=True, clean_queue=True):
        """Override to preserve original plan even when overtaking"""
        self._original_route = plan.copy() if plan else []
        result = super().set_global_plan(plan, stop_waypoint_creation, clean_queue)
        
        # Visualize the new plan if debugging is enabled
        if self._debug_trajectory:
            self._draw_plan_trajectory()
            
        return result

    def _check_overtake(self, waypoint, vehicle_list):
        """
        This method checks if there's a slow vehicle ahead that
        should be overtaken.

            :param waypoint: current waypoint of the agent
            :param vehicle_list: list of all the nearby vehicles
        """
        if self._overtaking:
            # Already in an overtaking maneuver, check if we should return to original lane
            ego_location = self._vehicle.get_location()
            
            # Check if we've passed the vehicle we're overtaking
            if self._vehicle_being_overtaken:
                overtaken_location = self._vehicle_being_overtaken.get_location()
                ego_transform = self._vehicle.get_transform()
                
                # Vector from our vehicle to the overtaken vehicle
                to_overtaken = overtaken_location - ego_location
                
                # Get our forward vector
                forward_vector = ego_transform.get_forward_vector()
                
                # Check if the overtaken vehicle is behind us (negative dot product)
                dot_product = forward_vector.x * to_overtaken.x + forward_vector.y * to_overtaken.y
                distance_to_overtaken = ego_location.distance(overtaken_location)
                
                # We've passed the vehicle if it's behind us and at a sufficient distance
                passed_vehicle = dot_product < 0 and distance_to_overtaken > self._min_distance_passed
                
                if passed_vehicle:
                    print(f"Passed vehicle, continuing in new lane for {self._overtake_completion_distance}m")
                    # We've passed the vehicle, but continue in this lane for a bit before returning
                    if self._return_point is None:
                        # Set a new return point that's further ahead
                        next_wp = waypoint
                        for _ in range(10):
                            next_wp = next_wp.next(self._overtake_completion_distance/10)[0]
                        self._return_point = next_wp.transform.location
                    
                    # Reset the vehicle being overtaken to avoid rechecking
                    self._vehicle_being_overtaken = None
            
            # If we've traveled far enough in the new lane after passing the vehicle
            if self._return_point and ego_location.distance(self._return_point) < 5 and self._return_to_original_route:
                print("Completed overtaking maneuver, returning to original lane")
                self._overtaking = False
                
                # Find a suitable waypoint on the original route to continue from
                self._rejoin_original_route()
                
            return

        if self._overtake_counter > 0:
            self._overtake_counter -= 1
            return

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
            # Check if left lane is free
            new_vehicle_state, _, _ = self._vehicle_obstacle_detected(
                vehicle_list, self._overtake_distance_threshold, up_angle_th=90, lane_offset=-1)
            
            if not new_vehicle_state:
                print("Slow vehicle ahead! Overtaking from the left")
                self._execute_lane_change('left', waypoint, left_wpt, vehicle)
                return

        # If left lane is not available or occupied, try right lane
        elif (right_turn == carla.LaneChange.Right or right_turn == carla.LaneChange.Both) and \
                waypoint.lane_id * right_wpt.lane_id > 0 and right_wpt.lane_type == carla.LaneType.Driving:
            # Check if right lane is free
            new_vehicle_state, _, _ = self._vehicle_obstacle_detected(
                vehicle_list, self._overtake_distance_threshold, up_angle_th=90, lane_offset=1)
            
            if not new_vehicle_state:
                print("Slow vehicle ahead! Overtaking from the right")
                self._execute_lane_change('right', waypoint, right_wpt, vehicle)
                return

    def _execute_lane_change(self, direction, current_waypoint, target_lane_waypoint, vehicle_to_overtake=None):
        """
        Execute a lane change maneuver by creating a new temporary route plan
        
        Args:
            direction: 'left' or 'right'
            current_waypoint: Current waypoint of the agent
            target_lane_waypoint: Waypoint in the target lane
            vehicle_to_overtake: The vehicle that we're trying to overtake
        """
        self._overtaking = True
        self._overtake_direction = direction
        self._overtake_counter = self._overtake_time
        self._original_waypoint = current_waypoint
        self._target_lane_id = target_lane_waypoint.lane_id
        self._vehicle_being_overtaken = vehicle_to_overtake
        self._return_point = None  # Reset return point, will be set after passing the vehicle
        
        # Create waypoints for the lane change maneuver
        route_plan = []
        
        # First add the immediate lane change waypoint
        next_wp = target_lane_waypoint
        route_plan.append((next_wp, RoadOption.CHANGELANERIGHT if direction == 'right' else RoadOption.CHANGELANELEFT))
        
        # Then continue in the new lane for a longer distance to ensure full overtaking
        for _ in range(20):  # Generate more waypoints to ensure we complete the overtake
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

    def _rejoin_original_route(self):
        """
        Find the best point on the original route to rejoin after overtaking
        without causing a U-turn.
        """
        if not self._original_route:
            print("Warning: No original route to rejoin!")
            return

        current_location = self._vehicle.get_location()
        current_waypoint = self._map.get_waypoint(current_location)
        vehicle_transform = self._vehicle.get_transform()
        
        # Get vehicle's forward vector
        forward_vector = vehicle_transform.get_forward_vector()
        
        # Find waypoints on the original route that are AHEAD of us
        candidate_waypoints = []
        for i, (wp, road_option) in enumerate(self._original_route):
            # Calculate vector from vehicle to waypoint
            wp_vector = wp.transform.location - current_location
            
            # Skip if the point is behind us (negative dot product with forward vector)
            # and not too far away (within reasonable distance)
            distance = current_location.distance(wp.transform.location)
            
            # Calculate dot product to determine if waypoint is ahead of us
            dot_product = forward_vector.x * wp_vector.x + forward_vector.y * wp_vector.y
            
            # Only consider waypoints that are ahead of us (positive dot product)
            # and not too far away (within reasonable distance)
            if dot_product > 0 and distance < 120 and distance > 50:
                # Store the waypoint, its index, and the distance
                candidate_waypoints.append((wp, road_option, i, distance))
        
        if not candidate_waypoints:
            print("Warning: No suitable waypoints found ahead on original route!")
            # Fall back to closest waypoint if no candidates ahead
            return self._fallback_rejoin()
        
        # Sort by distance (closest first)
        candidate_waypoints.sort(key=lambda x: x[3])
        
        # Select the closest waypoint ahead of us
        best_wp, road_option, idx, _ = candidate_waypoints[0]
        
        # Create a new plan from our current position to the best waypoint, then include
        # the rest of the original route from that point
        new_plan = []
        
        # First create a connection path from our current waypoint to the chosen route waypoint
        connection_path = self.trace_route(current_waypoint, best_wp)
        if connection_path:
            new_plan.extend(connection_path)
            
            # Visualize the connection path
            if self._debug_trajectory:
                self._draw_connection_trajectory(connection_path)
        
        # Then add the rest of the original route from the best waypoint onwards
        new_plan.extend(self._original_route[idx:])
        
        # Set the new global plan
        self._local_planner.set_global_plan(new_plan, stop_waypoint_creation=False)
        print(f"Rejoining original route with {len(new_plan)} waypoints")
        
        # Visualize the rejoined path
        if self._debug_trajectory:
            self._draw_plan_trajectory()

    def _fallback_rejoin(self):
        """Fallback method when no suitable waypoints are found ahead."""
        print("Using fallback method to rejoin route")
        
        current_location = self._vehicle.get_location()
        current_waypoint = self._map.get_waypoint(current_location)
        
        # Find closest waypoint by distance
        min_distance = float('inf')
        closest_idx = 0
        
        for i, (wp, _) in enumerate(self._original_route):
            distance = current_location.distance(wp.transform.location)
            if distance < min_distance:
                min_distance = distance
                closest_idx = i
        
        # Get the closest waypoint
        closest_wp, road_option = self._original_route[closest_idx]
        
        # Create path to closest waypoint
        connection_path = self.trace_route(current_waypoint, closest_wp)
        
        # Create new plan: connection path + remaining original route
        new_plan = []
        if connection_path:
            new_plan.extend(connection_path)
        
        # Add remaining waypoints from original route
        new_plan.extend(self._original_route[closest_idx:])
        
        # Set the new global plan
        self._local_planner.set_global_plan(new_plan, stop_waypoint_creation=False)

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
            color = self._regular_route_color
            if self._overtaking:
                color = self._overtaking_color
                
            line = self._debug.draw_line(
                prev_loc,
                loc,
                thickness=self._line_thickness,
                color=color,
                life_time=self._visualization_time
            )
            self._trajectory_points.append(line)
            
            # Draw a point at each waypoint
            point = self._debug.draw_point(
                loc,
                size=self._point_size,
                color=color,
                life_time=self._visualization_time
            )
            self._trajectory_points.append(point)
            
            prev_loc = loc
            
        # Try to highlight next target waypoint - handle different LocalPlanner implementations
        try:
            # Check if waypoint buffer exists (newer CARLA versions)
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
            # Alternative: try to access target waypoint directly (older CARLA versions)
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
            # If we can't find a target waypoint, just highlight the first waypoint in the queue
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
            # If anything goes wrong with visualization, just log it and continue
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

    def _draw_connection_trajectory(self, connection_path):
        """
        Visualizes the connection path when rejoining the original route.
        
        Args:
            connection_path: List of (waypoint, road_option) tuples for the connection path
        """
        if not connection_path:
            return
            
        # Draw the connection path
        prev_loc = self._vehicle.get_location()
        prev_loc.z += self._height_offset
        
        for i, (waypoint, _) in enumerate(connection_path):
            loc = waypoint.transform.location
            loc.z += self._height_offset
            
            line = self._debug.draw_line(
                prev_loc,
                loc,
                thickness=self._line_thickness,
                color=self._connection_path_color,
                life_time=self._visualization_time
            )
            self._trajectory_points.append(line)
            
            point = self._debug.draw_point(
                loc,
                size=self._point_size,
                color=self._connection_path_color,
                life_time=self._visualization_time
            )
            self._trajectory_points.append(point)
            
            prev_loc = loc

    def _clear_trajectory_visualization(self):
        """
        Clears all trajectory visualization objects.
        """
        # With persistent lifetime we would need to clear manually
        # With short lifetimes they'll disappear on their own
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

        # Continue with normal car following behavior if not actively overtaking
        if not self._overtaking:
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

        # 3: Intersection behavior or continue overtaking
        if not control:
            if self._overtaking:
                # If overtaking, maintain higher speed and focus on following the temporary plan
                target_speed = min([
                    self._behavior.max_speed * 1.1,  # Go slightly faster during overtaking
                    self._speed_limit + 5  # Allow slight speeding during overtaking
                ])
                self._local_planner.set_speed(target_speed)
                control = self._local_planner.run_step(debug=debug)
            elif self._incoming_waypoint.is_junction and (self._incoming_direction in [RoadOption.LEFT, RoadOption.RIGHT]):
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

    # Add a new method to check if a destination can be reached without returning to the original route
    def set_destination_without_route_constraint(self, end_waypoint):
        """
        Sets a destination without requiring the vehicle to return to the original route.
        
        Args:
            end_waypoint: Final waypoint to reach
        """
        self._return_to_original_route = False
        
        # Get current waypoint
        current_location = self._vehicle.get_location()
        start_waypoint = self._map.get_waypoint(current_location)
        
        # Generate a path to the destination
        direct_path = self.trace_route(start_waypoint, end_waypoint)
        
        if direct_path:
            self._local_planner.set_global_plan(direct_path, stop_waypoint_creation=False)
            print(f"Set direct path to destination with {len(direct_path)} waypoints")
            return True
        return False

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

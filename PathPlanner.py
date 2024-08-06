from typing import List, Tuple
from scipy.interpolate import splprep, splev
import numpy as np

from GeographicUtils import GeographicUtils
from ObstacleAvoidance import ObstacleAvoidance
from Point import Point,RedZone
from Visualizer import Visualizer


class PathPlanner:
    def __init__(self, start: Point, goal: Point, red_zones: List[RedZone],boundary_points:List[Point], step_size_km: float = 0.01):
        self.start = start
        self.goal = goal
        self.red_zones = red_zones
        self.step_size_km = step_size_km
        self.obstacle_avoidance = ObstacleAvoidance(red_zones,boundary_points)

    def generate_waypoints(self, start: Point, goal: Point) -> List[Point]:
        waypoints = [start]
        current_location = start
        
        while GeographicUtils.haversine(current_location, goal) > self.step_size_km:
            
            bearing = GeographicUtils.calculate_bearing(current_location, goal)
            current_location = GeographicUtils.point_with_bearing(current_location, self.step_size_km * 1000, bearing)
            
            waypoints.append(current_location)
            
        waypoints.append(goal)
        return waypoints
    
    def generate_complete_path_updated(self, drone_location: Point, middle_points: List[Point], target_location: Point) -> Tuple[List[Point], int]:
        complete_path = []

        # Start from the drone location
        current_location = drone_location

        # Function to find the closest middle point
        def find_closest_point(current_loc: Point, points: List[Point]) -> Point:
            closest_point = None
            shortest_distance = float('inf')
            for point in points:
                distance = GeographicUtils.haversine(current_loc, point)
                if distance < shortest_distance:
                    shortest_distance = distance
                    closest_point = point
            return closest_point

        remaining_middle_points = middle_points.copy()
        while remaining_middle_points:
            closest_point = find_closest_point(current_location, remaining_middle_points)
            remaining_middle_points.remove(closest_point)
            
            # Generate path to the closest middle point
            path_segment = self.generate_waypoints(current_location, closest_point)
            
            # Add to the complete path, excluding the last point to avoid duplicates
            complete_path += path_segment[:-1]
            
            # Update the current location
            current_location = closest_point
        
        if not self.obstacle_avoidance.path_is_clear_of_red_zones(complete_path):
            last_middle_point_index = len(complete_path)
        else:
            last_middle_point_index = None

        # Generate the final segment from the last middle point to the target location
        final_segment = self.generate_waypoints(current_location, target_location)
        
        # Add the final segment to the complete path
        complete_path += final_segment

        return complete_path, last_middle_point_index
    
    def get_points_around_middle_point(self, zone_details: RedZone, last_middle_point: Point, current_bearing: float) -> Tuple[Point, Point]:
        red_zone_radius = zone_details.radius
        
        initial_dist = 10 + zone_details.radius / 1.5
        decrement_step = 10  # Distance decrement step
        min_dist = 20  # Minimum allowable distance
        
        dist = initial_dist
        point_right_initial = GeographicUtils.point_with_bearing(zone_details.center, red_zone_radius + initial_dist, current_bearing + 90)
        point_left_initial = GeographicUtils.point_with_bearing(zone_details.center, red_zone_radius + initial_dist, current_bearing - 90)
        
        right_clear = False
        left_clear = False

        while dist >= min_dist:
            if not right_clear:
                point_right = GeographicUtils.point_with_bearing(zone_details.center, red_zone_radius + dist, current_bearing + 90)
                path_right = self.generate_waypoints(last_middle_point, point_right)
                right_clear = self.obstacle_avoidance.is_path_valid(path_right)
            
            if not left_clear:
                point_left = GeographicUtils.point_with_bearing(zone_details.center, red_zone_radius + dist, current_bearing - 90)
                path_left = self.generate_waypoints(last_middle_point, point_left)
                left_clear = self.obstacle_avoidance.is_path_valid(path_left)

            if right_clear and left_clear:
                return point_right, point_left
            
            dist -= decrement_step
        
        # If all distances fail, return the points with the smallest distance checked
        if not right_clear:
            point_right = point_right_initial
            print("Right point not clear given the initial distance{}".format(initial_dist))
        if not left_clear:
            point_left = point_left_initial
            print("Left point not clear given the initial distance{}".format(initial_dist))
        
        return point_right, point_left
    
    def get_preferred_and_alternative_points(self, reference_point: Point, point_right: Point, point_left: Point) -> Tuple[Point, Point]:
        
        if not self.obstacle_avoidance.is_point_valid(point_right):
            return point_left, point_left
        if not self.obstacle_avoidance.is_point_valid(point_left):
            return point_right, point_right
        
        distance_to_right = GeographicUtils.haversine(reference_point, point_right)
        distance_to_left = GeographicUtils.haversine(reference_point, point_left)
        
        preferred_point = point_right if distance_to_right <= distance_to_left else point_left
        alternative_point = point_left if preferred_point == point_right else point_right
        
        return preferred_point, alternative_point

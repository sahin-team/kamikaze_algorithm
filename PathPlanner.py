from typing import List, Tuple
from scipy.interpolate import splprep, splev
import numpy as np

from GeographicUtils import GeographicUtils
from ObstacleAvoidance import ObstacleAvoidance
from Point import Point
from RedZone import RedZone


class PathPlanner:
    def __init__(self, start: Point, goal: Point, red_zones: List[RedZone], step_size_km: float = 0.01):
        self.start = start
        self.goal = goal
        self.red_zones = red_zones
        self.step_size_km = step_size_km

    def generate_waypoints(self, start: Point, goal: Point) -> List[Point]:
        waypoints = [start]
        current_location = start
        
        while GeographicUtils.haversine(current_location.lat, current_location.lon, goal.lat, goal.lon) > self.step_size_km:
            lat1, lon1 = current_location.lat, current_location.lon
            lat2, lon2 = goal.lat, goal.lon
            
            bearing = GeographicUtils.calculate_bearing(lat1, lon1, lat2, lon2)
            new_lat, new_lon = GeographicUtils.point_with_bearing(lat1, lon1, self.step_size_km * 1000, bearing)
            
            current_location = Point(new_lat, new_lon)
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
                distance = GeographicUtils.haversine(current_loc.lat, current_loc.lon, point.lat, point.lon)
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
        
        if not ObstacleAvoidance.path_is_clear_of_red_zones(complete_path, self.red_zones):
            last_middle_point_index = len(complete_path)
        else:
            last_middle_point_index = None

        # Generate the final segment from the last middle point to the target location
        final_segment = self.generate_waypoints(current_location, target_location)
        
        # Add the final segment to the complete path
        complete_path += final_segment

        return complete_path, last_middle_point_index
    
    def get_points_around_middle_point(self, zone_details: RedZone, last_middle_point: Point, current_bearing: float) -> Tuple[Point, Point]:
        lat, lon = zone_details.center.lat, zone_details.center.lon
        red_zone_radius = zone_details.radius
        
        initial_dist = 10 + zone_details.radius / 1.5
        decrement_step = 10  # Distance decrement step
        min_dist = 20  # Minimum allowable distance
        
        dist = initial_dist
        point_right_initial = GeographicUtils.point_with_bearing(lat, lon, red_zone_radius + initial_dist, current_bearing + 90)
        point_left_initial = GeographicUtils.point_with_bearing(lat, lon, red_zone_radius + initial_dist, current_bearing - 90)
        
        point_right_initial = Point(point_right_initial[0], point_right_initial[1])
        point_left_initial = Point(point_left_initial[0], point_left_initial[1])
        
        right_clear = False
        left_clear = False

        while dist >= min_dist:
            if not right_clear:
                point_right = GeographicUtils.point_with_bearing(lat, lon, red_zone_radius + dist, current_bearing + 90)
                point_right = Point(point_right[0], point_right[1])
                path_right = self.generate_waypoints(last_middle_point, point_right)
                right_clear = ObstacleAvoidance.path_is_clear_of_red_zones(path_right, self.red_zones)
            
            if not left_clear:
                point_left = GeographicUtils.point_with_bearing(lat, lon, red_zone_radius + dist, current_bearing - 90)
                point_left = Point(point_left[0], point_left[1])
                path_left = self.generate_waypoints(last_middle_point, point_left)
                left_clear = ObstacleAvoidance.path_is_clear_of_red_zones(path_left, self.red_zones)

            if right_clear and left_clear:
                return point_right, point_left
            elif right_clear and not left_clear:
                dist -= decrement_step
                continue
            elif left_clear and not right_clear:
                dist -= decrement_step
                continue
            
            dist -= decrement_step
        
        # If all distances fail, return the points with the smallest distance checked
        if not right_clear:
            point_right = point_right_initial
        if not left_clear:
            point_left = point_left_initial
        
        return point_right, point_left
    
    def get_preferred_and_alternative_points(self, reference_point: Point, point_right: Point, point_left: Point) -> Tuple[Point, Point]:
        distance_to_right = GeographicUtils.haversine(reference_point.lat, reference_point.lon, point_right.lat, point_right.lon)
        distance_to_left = GeographicUtils.haversine(reference_point.lat, reference_point.lon, point_left.lat, point_left.lon)
        
        preferred_point = point_right if distance_to_right <= distance_to_left else point_left
        alternative_point = point_left if preferred_point == point_right else point_right
        
        return preferred_point, alternative_point

    def optimize_path(self, path: List[Point]) -> List[Point]:
        optimized_path = [path[0]]
        for i in range(1, len(path)-1):
            if ObstacleAvoidance.path_is_clear_of_red_zones(self.generate_waypoints(optimized_path[-1],path[i+1]), self.red_zones):
                optimized_path.append(path[i])
        optimized_path.append(path[-1])
        return optimized_path

    def smooth_path(self, path: List[Point]) -> List[Point]:
        latitudes = [point.lat for point in path]
        longitudes = [point.lon for point in path]

        tck, u = splprep([latitudes, longitudes], s=2)
        u_new = np.linspace(u.min(), u.max(), len(path) * 10)
        smoothed_coordinates = splev(u_new, tck)

        smoothed_path = [Point(lat, lon) for lat, lon in zip(smoothed_coordinates[0], smoothed_coordinates[1])]
        return smoothed_path

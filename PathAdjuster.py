
from typing import List, Tuple
from GeographicUtils import GeographicUtils
from Point import Point


class PathAdjuster:
    MAX_TURN_ANGLE = 10
    
    def adjust_initial_path(self, current_yaw: float, path: List[Point], max_turn_angle: float) -> Tuple[List[Point], List[Point]]:
        if len(path) < 2:
            return path, path  # Not enough points to adjust

        initial_bearing = GeographicUtils.calculate_bearing(path[0].lat, path[0].lon, path[1].lat, path[1].lon)
        print(f"Initial bearing: {initial_bearing}")
        
        yaw_diff = self.calculate_yaw_difference(current_yaw, initial_bearing)

        if abs(yaw_diff) > max_turn_angle:
            print("Adjusting initial path...")
            path_right = self.generate_adjusted_path(path[0], current_yaw, initial_bearing, max_turn_angle, 1)
            path_left = self.generate_adjusted_path(path[0], current_yaw, initial_bearing, max_turn_angle, -1)
            
            return path_right, path_left
        else:
            print("Initial path is valid")
            return [path[0], path[1]], [path[0], path[1]]

    def generate_adjusted_path(self, start_point: Point, current_yaw: float, target_bearing: float, 
                               max_turn_angle: float, direction: int) -> List[Point]:
        path = [start_point]
        current_bearing = current_yaw

        while abs(self.calculate_yaw_difference(current_bearing, target_bearing)) > max_turn_angle:
            current_bearing = (current_bearing + direction * max_turn_angle) % 360
            new_point = self.calculate_new_point(path[-1], current_bearing)
            path.append(new_point)

        # Add final point in the direction of the target
        final_point = self.calculate_new_point(path[-1], target_bearing)
        path.append(final_point)

        return path

    def calculate_yaw_difference(self, current_yaw: float, target_bearing: float) -> float:
        diff = target_bearing - current_yaw
        return (diff + 180) % 360 - 180

    def calculate_new_point(self, start: Point, bearing: float) -> Point:
        # This method should calculate a new point at a fixed distance in the given bearing
        # The distance could be a constant or based on the drone's speed and update rate
        distance = 0.01  # Example: 100 meters (adjust as needed)
        new_lat, new_lon = GeographicUtils.calculate_new_coordinates(start.lat, start.lon, bearing, distance)
        return Point(new_lat, new_lon)

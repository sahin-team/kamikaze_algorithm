
from typing import List
from GeographicUtils import GeographicUtils
from ObstacleAvoidance import ObstacleAvoidance
from Point import Point
from RedZone import RedZone
from Visualizer import Visualizer
from PathPlanner import PathPlanner


class DroneNavigator:
    def __init__(self, start: Point, goal: Point, red_zones: List[RedZone], max_iterations: int = 3):
        self.start = start
        self.goal = goal
        self.red_zones = red_zones
        self.max_iterations = max_iterations
        self.path_planner = PathPlanner(start, goal, red_zones)

    def generate_path(self) -> List[Point]:
        path = self.path_planner.generate_waypoints(self.start, self.goal)
        path_clear = ObstacleAvoidance.path_is_clear_of_red_zones(path, self.red_zones)
        iteration = 0
        self.middle_points_list = []
        preferred_point,alternative_point = None,None
        last_middle_point_index = None
        

        while not path_clear and iteration < self.max_iterations:
            # Implement obstacle avoidance logic here
            self.middle_point, zone_details = ObstacleAvoidance.find_first_red_zone_point(path, self.red_zones)
            
            print(f"Middle red zone point: {self.middle_point.lat,self.middle_point.lon}\nZone details: {zone_details.radius,zone_details.center.lat,zone_details.center.lon}")
            
            
            if last_middle_point_index:
                print("using last middle point index")
                current_bearing = GeographicUtils.calculate_bearing(path[last_middle_point_index-2].lat, path[last_middle_point_index-2].lon, path[last_middle_point_index-1].lat, path[last_middle_point_index-1].lon)
            else:
                current_bearing = GeographicUtils.calculate_bearing(path[-2].lat, path[-2].lon, path[-1].lat, path[-1].lon)
            
            print(f"Current bearing: {current_bearing}")
            
            if self.middle_points_list:
                point_right, point_left = self.path_planner.get_points_around_middle_point(zone_details, self.middle_points_list[-1], current_bearing)
            else:
                point_right, point_left = self.path_planner.get_points_around_middle_point(zone_details, self.goal, current_bearing)
            print(f"Point right: {point_right.lat,point_right.lon} - Point left: {point_left.lat,point_left.lon}")
            
            if preferred_point is not None:
                preferred_point,alternative_point = self.path_planner.get_preferred_and_alternative_points(preferred_point, point_right, point_left)
            else:
                preferred_point,alternative_point = self.path_planner.get_preferred_and_alternative_points(self.goal, point_right, point_left)
            
            print(f"Preferred point: {preferred_point.lat,preferred_point.lon} - Alternative point: {alternative_point.lat,alternative_point.lon}")
            
            for point in [preferred_point, alternative_point, preferred_point]:
                path, last_middle_point_index = self.path_planner.generate_complete_path_updated(self.start, [point] + self.middle_points_list, self.goal)
                path_clear = ObstacleAvoidance.path_is_clear_of_red_zones(path, self.red_zones)
                if path_clear:
                    break
            
            self.middle_points_list.append(preferred_point)
            print(f"Middle points list: {[(point.lat,point.lon) for point in self.middle_points_list]}")
                
                
            iteration += 1
            print("================================")
        if path_clear:
            print("Valid path found!")
            optimized_path = self.path_planner.optimize_path(path)
            print(f"Path optimized: {len(path)} points reduced to {len(optimized_path)} points")
            smoothed_path = self.path_planner.smooth_path(path)
            print("Path smoothed")
            
            if ObstacleAvoidance.path_is_clear_of_red_zones(smoothed_path, self.red_zones):
                print("Smoothed path is clear of red zones")
                return smoothed_path
            else:
                print("Smoothed path intersects red zones, using optimized path")
                return path
           
        else:
            print("Could not find a clear path within the maximum number of iterations")
            return path

    def navigate(self) -> List[Point]:
        path = self.generate_path()
        Visualizer.plot_path(path, self.red_zones, self.start, self.goal,[self.middle_point])
        return path

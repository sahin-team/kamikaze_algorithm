from typing import Generator, List
from GeographicUtils import GeographicUtils
from ObstacleAvoidance import ObstacleAvoidance
from PathAdjuster import PathAdjuster
from Point import Point,RedZone
from Visualizer import Visualizer
from PathPlanner import PathPlanner


class DroneNavigator:
    def __init__(self, start: Point, goal: Point, red_zones: List[RedZone], current_yaw : float, max_iterations: int = 2):
        self.start = start
        self.goal = goal
        self.red_zones = red_zones
        self.max_iterations = max_iterations
        self.path_planner = PathPlanner(start, goal, red_zones)
        self.current_yaw = current_yaw
        self.Max_turn_angle = 7

    def generate_path(self,start) -> List[Point]:
        path = self.path_planner.generate_waypoints(start, self.goal)
        path_clear = ObstacleAvoidance.path_is_clear_of_red_zones(path, self.red_zones)
        iteration = 0
        self.middle_points_list = []
        preferred_point,alternative_point = None,None
        last_middle_point_index = None
        

        while not path_clear and iteration < self.max_iterations:
            # Implement obstacle avoidance logic here
            self.middle_point, zone_details = ObstacleAvoidance.find_first_red_zone_point(path, self.red_zones)
            
            print(f"Middle red zone point: {self.middle_point.lat,self.middle_point.lon}\nZone details: {zone_details.radius,zone_details.center.lat,zone_details.center.lon}")
            
            
            last_idx = last_middle_point_index - 2 if last_middle_point_index else -2
            current_bearing = GeographicUtils.calculate_bearing(path[last_idx].lat, path[last_idx].lon, path[last_idx + 1].lat, path[last_idx + 1].lon)
            print(f"Current bearing: {current_bearing}")
            
                        
            target_point = self.middle_points_list[-1] if self.middle_points_list else self.goal
            point_right, point_left = self.path_planner.get_points_around_middle_point(zone_details, target_point, current_bearing)
            print(f"Point right: {point_right.lat, point_right.lon} - Point left: {point_left.lat, point_left.lon}")
            
            
            preferred_point, alternative_point = self.path_planner.get_preferred_and_alternative_points(preferred_point if preferred_point is not None else self.goal, point_right, point_left)

            print(f"Preferred point: {preferred_point.lat,preferred_point.lon} - Alternative point: {alternative_point.lat,alternative_point.lon}")
            
            for point in [preferred_point, alternative_point, preferred_point]:
                path, last_middle_point_index = self.path_planner.generate_complete_path_updated(start, [point] + self.middle_points_list, self.goal)
                path_clear = ObstacleAvoidance.path_is_clear_of_red_zones(path, self.red_zones)
                if path_clear:
                    break
            
            self.middle_points_list.append(preferred_point)
            print(f"Middle points list: {[(point.lat,point.lon) for point in self.middle_points_list]}")
                
                
            iteration += 1
            print("================================")
        if path_clear:
            print("Valid path found!")
            return path
        
        else:
            print("Could not find a clear path within the maximum number of iterations")
            return path

    def navigate(self) -> Generator[Point, None, None]:
        path = self.path_planner.generate_waypoints(self.start, self.goal)
        print(self.current_yaw)
        
        adjuster = PathAdjuster()
        right_turn, left_turn, adjusted = adjuster.adjust_initial_path(self.current_yaw, path, self.Max_turn_angle)
        
        if adjusted:
            right_path = right_turn + self.generate_path(right_turn[-1])
            left_path = left_turn + self.generate_path(left_turn[-1])

            # Choose the shortest clear path
            if len(right_path) < len(left_path) and ObstacleAvoidance.path_is_clear_of_red_zones(right_path, self.red_zones):
                path = right_path
            elif ObstacleAvoidance.path_is_clear_of_red_zones(left_path, self.red_zones):
                path = left_path
            else:
                path = right_path if ObstacleAvoidance.path_is_clear_of_red_zones(right_path, self.red_zones) else left_path
        else:
            path = self.generate_path(self.start)

        # Plot paths if adjusted; otherwise, plot the initial path
        # if adjusted:
        #     Visualizer.plot_path(right_path, self.red_zones, self.start, self.goal, left_path)
        
        # Visualizer.plot_path(path, self.red_zones, self.start, self.goal)
        
        for point in path:
            yield point
            


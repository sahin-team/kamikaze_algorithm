from typing import Generator, List,Tuple
from GeographicUtils import GeographicUtils
from ObstacleAvoidance import ObstacleAvoidance
from PathAdjuster import PathAdjuster
from Point import Point,RedZone
from Visualizer import Visualizer
from PathPlanner import PathPlanner


class DroneNavigator:
    def __init__(self, start: Point, goal: Point, red_zones: List[RedZone], current_yaw : float,boundary_points:List[Point], max_iterations: int = 3) -> None:
        self.start = start
        self.goal = goal
        self.red_zones = red_zones
        self.max_iterations = max_iterations
        self.path_planner = PathPlanner(start, goal, red_zones,boundary_points)
        self.obstacle_avoidance = ObstacleAvoidance(red_zones, boundary_points)
        self.current_yaw = current_yaw
        self.Max_turn_angle = 7
        self.boundary_points = boundary_points

    def generate_path(self,start) -> Tuple[List[Point],List[Point]]:
        path = self.path_planner.generate_waypoints(start, self.goal)
        path_clear = self.obstacle_avoidance.path_is_clear_of_red_zones(path)
        iteration = 0
        middle_points_list = []
        preferred_point,alternative_point = None,None
        last_middle_point_index = None
        

        while not path_clear and iteration < self.max_iterations:
            # Implement obstacle avoidance logic here
            _, zone_details = self.obstacle_avoidance.find_first_red_zone_point(path)
            
            print(f"Zone details: {zone_details.radius,zone_details.center.lat,zone_details.center.lon}")
            
            
            last_idx = last_middle_point_index - 2 if last_middle_point_index else -2
            current_bearing = GeographicUtils.calculate_bearing(path[last_idx], path[last_idx + 1])
            print(f"Current bearing: {current_bearing}")
            
                        
            target_point = middle_points_list[-1] if middle_points_list else self.goal
            point_right, point_left = self.path_planner.get_points_around_middle_point(zone_details, target_point, current_bearing)
            print(f"Point right: {point_right.lat, point_right.lon} - Point left: {point_left.lat, point_left.lon}")
            
            
            preferred_point, alternative_point = self.path_planner.get_preferred_and_alternative_points(preferred_point if preferred_point is not None else self.goal, point_right, point_left)

            print(f"Preferred point: {preferred_point.lat,preferred_point.lon} - Alternative point: {alternative_point.lat,alternative_point.lon}")
            
            for point in [preferred_point, alternative_point, preferred_point]:
                path, last_middle_point_index = self.path_planner.generate_complete_path_updated(start, [point] + middle_points_list, self.goal)
                path_clear = self.obstacle_avoidance.path_is_clear_of_red_zones(path)
                if path_clear:
                    break
            
            middle_points_list.append(preferred_point)
            print(f"Middle points list: {[(point.lat,point.lon) for point in middle_points_list]}")
                
                
            iteration += 1
            print("================================")
        if path_clear:
            print("Valid path found!")
            return path,middle_points_list
        
        else:
            print("Could not find a clear path within the maximum number of iterations")
            return path,middle_points_list

    def navigate(self) -> Generator[Point, None, None]:
        path = self.path_planner.generate_waypoints(self.start, self.goal)
        print(self.current_yaw)
        
        adjuster = PathAdjuster()
        right_turn, left_turn, adjusted = adjuster.adjust_initial_path(self.current_yaw, path, self.Max_turn_angle)

        if adjusted:
            # Generate paths after the initial adjustments
            right_path, right_middle_points = self.generate_path(right_turn[-1])
            left_path, left_middle_points = self.generate_path(left_turn[-1])
            
            # Combine initial turns with the generated paths
            full_right_path = right_turn + right_path
            full_left_path = left_turn + left_path

            # Choose the shortest clear path
            if len(full_right_path) < len(full_left_path) and self.obstacle_avoidance.path_is_clear_of_red_zones(full_right_path):
                path,middle_points = full_right_path,right_middle_points
            elif self.obstacle_avoidance.path_is_clear_of_red_zones(full_left_path):
                path,middle_points = full_left_path,left_middle_points
            else:
                path = full_right_path if self.obstacle_avoidance.path_is_clear_of_red_zones(full_right_path) else full_left_path
                middle_points = right_middle_points if self.obstacle_avoidance.path_is_clear_of_red_zones(full_right_path) else left_middle_points
        else:
            path, middle_points = self.generate_path(self.start)

        # Adjust the distance between waypoints while preserving middle points
        path = GeographicUtils.increase_distance(path, middle_points,10)
    

        # Plot paths if adjusted; otherwise, plot the initial path
        # if adjusted:
        #     Visualizer.plot_path(right_path, self.red_zones, self.start, self.goal, left_path)
        
        
        
        # if not self.obstacle_avoidance.is_path_valid(path):
        #     print(f"path is not valid: ")
        
        
        Visualizer.plot_path(path, self.red_zones, self.start, self.goal,self.boundary_points)
        
        print(self.obstacle_avoidance.is_point_valid(self.start))
        for point in path:
            yield point
            
            


from functions import find_first_red_zone_point, haversine, path_is_clear_of_red_zones, find_middle_red_zone_point, generate_complete_path, generate_waypoints, get_points_around_middle_point, plot_path
from new_functions import generate_complete_path_updated, get_current_bearing, get_points_around_middle_point1, get_points_around_middle_point2, get_preferred_and_alternative_points, plot_path_with_two_points

import numpy as np
from scipy.interpolate import splprep, splev

# Drone and QR code locations
# drone_location = {"lat": 40.23107, "lon": 29.00900}
drone_location = {"lat": 40.22997, "lon": 28.99953}
# drone_location = {"lat": 40.23008, "lon": 29.00499}
# drone_location = {"lat": 40.23313, "lon": 29.00429}
# drone_location = {"lat": 40.22999, "lon": 29.00828}
drone_location = {"lat": 40.23120, "lon": 29.00003}


# qr_code_location = {"lat": 40.23428, "lon": 29.00027}
qr_code_location = {"lat": 40.23008, "lon": 29.00499}

redzones = [
    {"id": 0, "lat": 40.23260922, "lon": 29.00573015, "radius": 50},
    {"id": 1, "lat": 40.23351019, "lon": 28.99976492, "radius": 50},
    {"id": 2, "lat": 40.23105297, "lon": 29.00744677, "radius": 75},
    {"id": 3, "lat": 40.23090554, "lon": 29.00221109, "radius": 170}
]

# Convert coordinates to tuple
start = (drone_location["lat"], drone_location["lon"])
goal = (qr_code_location["lat"], qr_code_location["lon"])

def smooth_path(path, smoothing=0.1):
    x, y = zip(*path)
    tck, u = splprep([x, y], s=smoothing)
    new_points = splev(np.linspace(0, 1, len(path)), tck)
    return list(zip(new_points[0], new_points[1]))

def optimize_path(path, redzones):
    """Simple path optimization by removing unnecessary waypoints."""
    optimized_path = [path[0]]
    for i in range(1, len(path) - 1):
        if not path_is_clear_of_red_zones([optimized_path[-1], path[i+1]], redzones):
            optimized_path.append(path[i])
    optimized_path.append(path[-1])
    return optimized_path

def main():
    step_size_km = 0.01
    path = generate_waypoints(start, goal, step_size_km)
    path_clear = path_is_clear_of_red_zones(path, redzones)
    middle_points_list = []
    max_iterations = 10  # Increased from 5 to 10 for more flexibility
    iteration = 0
    preferred_point = None

    while not path_clear and iteration < max_iterations:
        middle_point, zone_details = find_first_red_zone_point(path, redzones)
        if middle_point:
            print(f"Iteration {iteration + 1}")
            print(f"Middle red zone point: {middle_point}\nZone details: {zone_details}")
            current_bearing = get_current_bearing([path[-3], path[-2], path[-1]])
            print(f"Current bearing: {current_bearing}")
            if middle_points_list:
                point_right,point_left = get_points_around_middle_point1(zone_details, middle_points_list[-1], redzones,current_bearing)
            else:
                point_right,point_left = get_points_around_middle_point1(zone_details, start, redzones,current_bearing)
            
            if preferred_point is not None:
                preferred_point, alternative_point = get_preferred_and_alternative_points(preferred_point, point_right, point_left)
            else:
                preferred_point, alternative_point = get_preferred_and_alternative_points(goal, point_right, point_left)
            
            path, last_middle_point_index = generate_complete_path_updated(drone_location, [preferred_point] + middle_points_list, qr_code_location, step_size_km)
            path_clear = path_is_clear_of_red_zones(path, redzones)
            
            if not path_clear:
                path, last_middle_point_index = generate_complete_path_updated(drone_location, [alternative_point] + middle_points_list, qr_code_location, step_size_km)
                path_clear = path_is_clear_of_red_zones(path, redzones)
                if not path_clear:
                    path, last_middle_point_index = generate_complete_path_updated(drone_location, [preferred_point] + middle_points_list, qr_code_location, step_size_km)
            else:
                print("Preferred point used")
            
            middle_points_list.append(preferred_point)
            iteration += 1
        else:
            path_clear = True
        print("================================")

    if path_clear:
        print("Valid path found!")
        # Optimize the path
        optimized_path = optimize_path(path, redzones)
        print(f"Path optimized: {len(path)} points reduced to {len(optimized_path)} points")
        
        # Smooth the path
        smoothed_path = smooth_path(path)
        print("Path smoothed")
        # print(smoothed_path)
        plot_path_with_two_points(smoothed_path, drone_location, qr_code_location, redzones, (40.23,29.23), (40.23,29.23))
        # Final check to ensure smoothed path is still clear
        # if path_is_clear_of_red_zones(smoothed_path, redzones):
        #     print("Smoothed path is clear of red zones")
        #     plot_path_with_two_points(smoothed_path, drone_location, qr_code_location, redzones, None, None)
        # else:
        #     print("Smoothed path intersects red zones, using optimized path")
        #     plot_path_with_two_points(optimized_path, drone_location, qr_code_location, redzones, None, None)
    else:
        print("Could not find a clear path within the maximum number of iterations")
        plot_path_with_two_points(smooth_path(optimize_path(path,redzones)), drone_location, qr_code_location, redzones, (40.23,29.23), (40.23,29.23))

if __name__ == "__main__":
    main()
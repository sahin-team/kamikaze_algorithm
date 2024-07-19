from functions import find_first_red_zone_point, haversine,path_is_clear_of_red_zones,find_middle_red_zone_point, generate_complete_path, generate_waypoints, get_points_around_middle_point, plot_path
from new_functions import generate_complete_path_updated, get_current_bearing, get_points_around_middle_point2, get_preferred_and_alternative_points, plot_path_with_two_points

# Drone and QR code locations
# Coordinates of drone and target

# drone_location = {"lat": 40.22997, "lon": 28.99953} problem
# drone_location = {"lat": 40.23008, "lon": 29.00499}
# drone_location = {"lat": 40.23313, "lon": 29.00429}
# drone_location = {"lat": 40.22999, "lon": 29.00828}
# drone_location = {"lat": 40.23064, "lon": 29.00918}
drone_location = {"lat": 40.23107, "lon": 29.00900}
# drone_location = {"lat": 40.22972, "lon": 29.00894}
# drone_location = {"lat": 40.23392, "lon": 28.99619}



qr_code_location = {"lat": 40.23428, "lon": 29.00027}

redzones = [
    {"id": 0, "lat": 40.23260922, "lon": 29.00573015, "radius": 50},
    {"id": 1, "lat": 40.23351019, "lon": 28.99976492, "radius": 50},
    {"id": 2, "lat": 40.23105297, "lon": 29.00744677, "radius": 75},
    {"id": 3, "lat": 40.23090554, "lon": 29.00221109, "radius": 170}
]

# Convert coordinates to tuple
start = (drone_location["lat"], drone_location["lon"])
goal = (qr_code_location["lat"], qr_code_location["lon"])

def main():
    # Generate waypoints with a step size of 0.01 km
    
    step_size_km = 0.01
    path = generate_waypoints(start, goal, step_size_km)
    path_clear = path_is_clear_of_red_zones(path, redzones)
    middle_points_list = []
    count = 1
    point_right = (40.23284722988171, 29.00236334657081)
    point_left = (40.23284722988171, 29.00236334657081)
    preferred_point = None
    

    print(middle_points_list)
    while not path_clear and count < 5:
        middle_point,zone_details = find_first_red_zone_point(path, redzones)
        if middle_point:
            print(f"Middle red zone point: {middle_point}\nZone details: {zone_details}")
            current_bearing = get_current_bearing([path[-3], path[-2],path[-1]])
            print(f"Current bearing: {current_bearing}")
            point_right,point_left = get_points_around_middle_point2(zone_details, zone_details["radius"], redzones,current_bearing)
            
            if preferred_point is not None:
                preferred_point, alternative_point = get_preferred_and_alternative_points(preferred_point, point_right, point_left)
            else:
                preferred_point, alternative_point = get_preferred_and_alternative_points(goal, point_right, point_left)
                        
                
            path, last_middle_point_index = generate_complete_path_updated(drone_location,[preferred_point]+middle_points_list, qr_code_location, step_size_km)
            path_clear = path_is_clear_of_red_zones(path, redzones)
            
            
            if not path_clear:
                path, last_middle_point_index = generate_complete_path_updated(drone_location, [alternative_point] + middle_points_list, qr_code_location, step_size_km)
                path_clear = path_is_clear_of_red_zones(path, redzones)
                if not path_clear:
                    path, last_middle_point_index = generate_complete_path_updated(drone_location,[preferred_point]+middle_points_list, qr_code_location, step_size_km)
                    
            else:
                print("preferred point used")
                
            middle_points_list.append(preferred_point)
            count += 1       
        else:
            path_clear = True
        print("================================")
            
    print(middle_points_list)
    print("als")
    path_clear = path_is_clear_of_red_zones(path, redzones)
    print(path_clear)
    plot_path_with_two_points(path, drone_location, qr_code_location, redzones, point_right,point_left)
    

if __name__ == "__main__":
    main() 


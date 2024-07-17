from functions import find_first_red_zone_point,path_is_clear_of_red_zones,find_middle_red_zone_point, generate_complete_path, generate_waypoints, get_points_around_middle_point, plot_path

# Drone and QR code locations
# Coordinates of drone and target

# drone_location = {"lat": 40.23008, "lon": 29.00499}
# drone_location = {"lat": 40.22997, "lon": 28.99953}
# drone_location = {"lat": 40.23008, "lon": 29.00499}
# drone_location = {"lat": 40.23313, "lon": 29.00429}
drone_location = {"lat": 40.22999, "lon": 29.00828}
# drone_location = {"lat": 40.22782, "lon": 29.00202}


qr_code_location = {"lat": 40.23323, "lon": 29.002}

redzones = [
    {"id": 0, "lat": 40.23260922, "lon": 29.00573015, "radius": 50},
    {"id": 1, "lat": 40.23351019, "lon": 28.99976492, "radius": 50},
    {"id": 2, "lat": 40.23105297, "lon": 29.00744677, "radius": 75},
    {"id": 3, "lat": 40.23090554, "lon": 29.00221109, "radius": 150}
]


# Convert coordinates to tuple
start = (drone_location["lat"], drone_location["lon"])
goal = (qr_code_location["lat"], qr_code_location["lon"])

def main():
    # Generate waypoints with a step size of 0.01 km
    
    step_size_km = 0.01
    path = generate_waypoints(start, goal, step_size_km)
    path_clear = path_is_clear_of_red_zones(path, redzones)
        
        
    while not path_clear:
        
        middle_red_zone_point = find_first_red_zone_point(path, redzones)
        if middle_red_zone_point[0]:
            print(f"Middle red zone point: {middle_red_zone_point}")
            point_right_or_left = get_points_around_middle_point(middle_red_zone_point[0], middle_red_zone_point[1]["radius"], redzones)
            path = generate_complete_path(drone_location, point_right_or_left, qr_code_location, step_size_km)
        else:
            point_right_or_left = (40.23284722988171, 29.00236334657081)
            path_clear = True
            
        path_clear = path_is_clear_of_red_zones(path, redzones)
        print(f"Path is clear of red zones: {path_clear}")

    plot_path(path, drone_location, qr_code_location, redzones, point_right_or_left)
    
main()

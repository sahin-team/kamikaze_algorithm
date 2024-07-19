import math

from matplotlib import pyplot as plt
from matplotlib.patches import Circle
from functions import generate_waypoints, haversine, in_red_zone, meters_to_degrees


def calculate_bearing(lat1, lon1, lat2, lon2):
    lat1_rad = math.radians(lat1)
    lon1_rad = math.radians(lon1)
    lat2_rad = math.radians(lat2)
    lon2_rad = math.radians(lon2)

    delta_lon = lon2_rad - lon1_rad

    x = math.sin(delta_lon) * math.cos(lat2_rad)
    y = math.cos(lat1_rad) * math.sin(lat2_rad) - math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(delta_lon)

    initial_bearing = math.atan2(x, y)
    initial_bearing = math.degrees(initial_bearing)
    compass_bearing = (initial_bearing + 360) % 360

    return compass_bearing

def get_current_bearing(path):
    if len(path) < 2:
        raise ValueError("Path must contain at least two points to calculate bearing.")
    
    lat1, lon1 = path[0]
    lat2, lon2 = path[1]
    
    return calculate_bearing(lat1, lon1, lat2, lon2)




def point_with_bearing(lat, lon, distance, bearing):
    R = 6378137  # Radius of the Earth in meters

    bearing_rad = math.radians(bearing)
    lat_rad = math.radians(lat)
    lon_rad = math.radians(lon)

    new_lat_rad = math.asin(math.sin(lat_rad) * math.cos(distance / R) +
                            math.cos(lat_rad) * math.sin(distance / R) * math.cos(bearing_rad))

    new_lon_rad = lon_rad + math.atan2(math.sin(bearing_rad) * math.sin(distance / R) * math.cos(lat_rad),
                                       math.cos(distance / R) - math.sin(lat_rad) * math.sin(new_lat_rad))

    new_lat = math.degrees(new_lat_rad)
    new_lon = math.degrees(new_lon_rad)

    return new_lat, new_lon


def get_points_around_middle_point1(middle_point, red_zone_radius,redzones,current_bearing):
    lat = middle_point["lat"]
    lon = middle_point["lon"]
    
    # Calculate points to the right (bearing 90 degrees) and left (bearing 270 degrees) of the middle point
    point_right = point_with_bearing(lat, lon, red_zone_radius+50, current_bearing + 90)
    point_left = point_with_bearing(lat, lon, red_zone_radius+50, current_bearing - 90)
    
    if in_red_zone(point_right[0], point_right[1], redzones):
        return point_left
    else:
        return point_right

def plot_path_with_two_points(path, drone_location, qr_code_location, redzones,point_right,point_left):
    fig, ax = plt.subplots()
    
    # Plot red zones
    for zone in redzones:
        lat, lon, radius = zone["lat"], zone["lon"], zone["radius"]
        radius_lat, radius_lon = meters_to_degrees(radius, lat)
        circle = Circle((lon, lat), radius_lon, color='red', alpha=0.5)
        ax.add_patch(circle)

    # Plot the drone and target
    ax.plot(drone_location["lon"], drone_location["lat"], marker='o', markersize=10, color='blue', label='Drone')
    ax.plot(qr_code_location["lon"], qr_code_location["lat"], marker='x', markersize=10, color='green', label='Target')
    ax.plot(point_right[1], point_right[0], marker='o', markersize=10, color='yellow', label='Middle Point')
    ax.plot(point_left[1], point_left[0], marker='o', markersize=10, color='yellow', label='Middle Point')
    
    # Plot the path
    path_lons = [point[1] for point in path]
    path_lats = [point[0] for point in path]
    ax.plot(path_lons, path_lats, marker='o', color='orange', label='Path')
    
    
    # ax.set_xlim(min_lon - lon_buffer, max_lon + lon_buffer)
    # ax.set_ylim(min_lat - lat_buffer, max_lat + lat_buffer)
    # Set limits and labels
    min_lat = min(zone["lat"] - meters_to_degrees(zone["radius"], zone["lat"])[0] for zone in redzones)
    max_lat = max(zone["lat"] + meters_to_degrees(zone["radius"], zone["lat"])[0] for zone in redzones)
    min_lon = min(zone["lon"] - meters_to_degrees(zone["radius"], zone["lat"])[1] for zone in redzones)
    max_lon = max(zone["lon"] + meters_to_degrees(zone["radius"], zone["lat"])[1] for zone in redzones)

    
    
    buffer_factor = 0.8  # Adjust the buffer factor as needed
    lat_buffer = buffer_factor * (max_lat - min_lat)
    lon_buffer = buffer_factor * (max_lon - min_lon)
    ax.set_xlim(min_lon - lon_buffer, max_lon + lon_buffer)
    ax.set_ylim(min_lat - lat_buffer, max_lat + lat_buffer)
    # ax.set_xlim(min_lon, max_lon)
    # ax.set_ylim(min_lat, max_lat)
    ax.set_xlabel("lon")
    ax.set_ylabel("lat")
    ax.set_title("Red Zones with Drone and Target")
    ax.legend()

    # Show plot
    plt.gca().set_aspect('equal', adjustable='box')
    plt.grid(True)
    plt.show()

def get_points_around_middle_point2(middle_point, red_zone_radius,redzones,current_bearing):
    lat = middle_point["lat"]
    lon = middle_point["lon"]
    
    dist = 18.5 + 70
    # Calculate points to the right (bearing 90 degrees) and left (bearing 270 degrees) of the middle point
    point_right = point_with_bearing(lat, lon, red_zone_radius+dist, current_bearing + 90)
    point_left = point_with_bearing(lat, lon, red_zone_radius+dist, current_bearing - 90)
    
    return point_right,point_left


def generate_complete_path_updated(drone_location, middle_points, target_location, step_size_km):
    complete_path = []
    last_middle_point_index = -1  # To store the index of the last middle point

    # Start from the drone location
    current_location = (drone_location["lat"], drone_location["lon"])

    # Function to find the closest middle point
    def find_closest_point(current_location, points):
        closest_point = None
        shortest_distance = float('inf')
        for point in points:
            distance = haversine(current_location[0], current_location[1], point[0], point[1])
            if distance < shortest_distance:
                shortest_distance = distance
                closest_point = point
        return closest_point

    while middle_points:
        closest_point = find_closest_point(current_location, middle_points)
        last_middle_point_index = middle_points.index(closest_point)  # Update the index of the last middle point
        middle_points.remove(closest_point)
        
        # Generate path to the closest middle point
        path_segment = generate_waypoints(
            current_location,
            (closest_point[0], closest_point[1]),
            step_size_km
        )
        
        # Add to the complete path, excluding the last point to avoid duplicates
        complete_path += path_segment[:-1]
        
        # Update the current location
        current_location = (closest_point[0], closest_point[1])

    # Generate the final segment from the last middle point to the target location
    final_segment = generate_waypoints(
        current_location,
        (target_location["lat"], target_location["lon"]),
        step_size_km
    )
    
    # Add the final segment to the complete path
    complete_path += final_segment

    return complete_path, last_middle_point_index


def get_preferred_and_alternative_points(reference_point, point_right, point_left):
    distance_to_right = haversine(reference_point[0], reference_point[1], point_right[0], point_right[1])
    distance_to_left = haversine(reference_point[0], reference_point[1], point_left[0], point_left[1])
    
    preferred_point = point_right if distance_to_right <= distance_to_left else point_left
    alternative_point = point_left if preferred_point == point_right else point_right
    print(f"From get preferred and alternative points: preferred_point: {preferred_point}, alternative_point: {alternative_point}")
    
    return preferred_point, alternative_point
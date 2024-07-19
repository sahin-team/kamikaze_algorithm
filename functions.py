import math

from matplotlib import pyplot as plt
from matplotlib.patches import Circle


def haversine(lat1, lon1, lat2, lon2):
    R = 6371  # Radius of the Earth in km
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = math.sin(dlat / 2) ** 2 + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(dlon / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    distance = R * c
    return distance


def in_red_zone(lat, lon, redzones):
    for zone in redzones:
        if haversine(lat, lon, zone["lat"], zone["lon"]) <= (zone["radius"] +20)/ 1000:  # radius in km
            return True
    return False

def meters_to_degrees(meters, lat):
    return meters / 111000, meters / (111000 * math.cos(math.radians(lat)))



def generate_waypoints(start, goal, step_size_km):
    waypoints = [start]
    current_location = start
    
    while haversine(current_location[0], current_location[1], goal[0], goal[1]) > step_size_km:
        lat1, lon1 = current_location
        lat2, lon2 = goal
        
        # Calculate the bearing
        bearing = math.atan2(
            math.sin(math.radians(lon2 - lon1)) * math.cos(math.radians(lat2)),
            math.cos(math.radians(lat1)) * math.sin(math.radians(lat2)) - 
            math.sin(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.cos(math.radians(lon2 - lon1))
        )
        
        # Calculate next waypoint
        new_lat = math.asin(math.sin(math.radians(lat1)) * math.cos(step_size_km / 6371) +
                            math.cos(math.radians(lat1)) * math.sin(step_size_km / 6371) * math.cos(bearing))
        
        new_lon = math.radians(lon1) + math.atan2(
            math.sin(bearing) * math.sin(step_size_km / 6371) * math.cos(math.radians(lat1)),
            math.cos(step_size_km / 6371) - math.sin(math.radians(lat1)) * math.sin(new_lat)
        )
        
        new_lat = math.degrees(new_lat)
        new_lon = math.degrees(new_lon)
        
        current_location = (new_lat, new_lon)
        waypoints.append(current_location)
        
    waypoints.append(goal)
    return waypoints


def find_first_red_zone_point(path, redzones):
    for point in path:
        for zone in redzones:
            if haversine(point[0], point[1], zone["lat"], zone["lon"]) <= (zone["radius"] +20) / 1000:  # radius in km
                return point, zone  # Return the first point in the red zone and the red zone details
    return None, None


def find_middle_red_zone_point(path, redzones):
    middle_red_zone_point = None
    middle_red_zone_details = None  # To store lat, lon, radius of the red zone
    in_red_zone_count = 0
    enter_red_zone_index = -1

    for i, point in enumerate(path):
        is_in_red_zone = False
        for zone in redzones:
            if haversine(point[0], point[1], zone["lat"], zone["lon"]) <= zone["radius"] / 1000:  # radius in km
                is_in_red_zone = True
                current_red_zone = zone  # Store the current red zone details
                break
        
        print(f"Point: {point}, In red zone: {is_in_red_zone}")

        if is_in_red_zone:
            if enter_red_zone_index == -1:
                enter_red_zone_index = i
            in_red_zone_count += 1
        elif in_red_zone_count > 0:
            middle_index = enter_red_zone_index + in_red_zone_count // 2
            middle_red_zone_point = path[middle_index]
            middle_red_zone_details = current_red_zone  # Store the red zone details
            in_red_zone_count = 0
            enter_red_zone_index = -1

    # If the red zone ends at the last point
    if in_red_zone_count > 0:
        middle_index = enter_red_zone_index + in_red_zone_count // 2
        middle_red_zone_point = path[middle_index]
        middle_red_zone_details = current_red_zone  # Store the red zone details

    return middle_red_zone_point, middle_red_zone_details


def plot_path(path, drone_location, qr_code_location, redzones,point_right_or_left):
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
    ax.plot(point_right_or_left[1], point_right_or_left[0], marker='o', markersize=10, color='yellow', label='Middle Point')
    
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

    
    
    buffer_factor = 0.1  # Adjust the buffer factor as needed
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


def get_points_around_middle_point(middle_point, red_zone_radius,redzones):
    lat, lon = middle_point
    # print(lat, lon)
    # print(red_zone_radius)
    # Calculate points to the right (bearing 90 degrees) and left (bearing 270 degrees) of the middle point
    point_right = point_with_bearing(lat, lon, red_zone_radius, 90)
    point_left = point_with_bearing(lat, lon, red_zone_radius, 270)
    
    if in_red_zone(point_right[0], point_right[1], redzones):
        return point_left
    else:
        return point_right


def generate_complete_path(drone_location, middle_points, target_location, step_size_km):
    complete_path = []

    # Start from the drone location
    current_location = (drone_location["lat"], drone_location["lon"])

    for middle_point in middle_points:
        # Generate path to the current middle point
        path_segment = generate_waypoints(
            current_location,
            (middle_point[0], middle_point[1]),
            step_size_km
        )
        
        # Add to the complete path, excluding the last point to avoid duplicates
        complete_path += path_segment[:-1]
        
        # Update the current location
        current_location = (middle_point[0], middle_point[1])

    # Generate the final segment from the last middle point to the target location
    final_segment = generate_waypoints(
        current_location,
        (target_location["lat"], target_location["lon"]),
        step_size_km
    )
    
    # Add the final segment to the complete path
    complete_path += final_segment

    return complete_path



def path_is_clear_of_red_zones(path, redzones):
    for point in path:
        if in_red_zone(point[0], point[1], redzones):
            # print(f"Point {point} is in a red zonexxxxxxxxxxxxxxxxxxmust return false")
            return False
        else:
            pass
            # print(f"Point {point} is not in a red zone")
    return True


def find_point_after_redzone_with_n_points(path, redzones, n):
    last_red_zone_index = -1
    
    # Find the last red zone point index
    for i, point in enumerate(path):
        if in_red_zone(point[0], point[1], redzones):
            last_red_zone_index = i
    
    # Check if we have found a red zone point and if there are enough points after it
    if last_red_zone_index == -1 or last_red_zone_index + n >= len(path):
        return None  # No red zone found or not enough points after the red zone
    
    # Return the point after the last red zone with n subsequent points
    return path[last_red_zone_index + n]
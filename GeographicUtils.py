import math
from typing import List, Set, Tuple

from Point import Point

EARTH_RADIUS_KM = 6371  # Radius of the Earth in km
EARTH_RADIUS_M = 6371000  # Radius of the Earth in meters

class GeographicUtils:
    @staticmethod
    def haversine(point1: Point, point2: Point) -> float:
        dlat = math.radians(point2.lat - point1.lat)
        dlon = math.radians(point2.lon - point1.lon)
        a = math.sin(dlat / 2) ** 2 + math.cos(math.radians(point1.lat)) * math.cos(math.radians(point2.lat)) * math.sin(dlon / 2) ** 2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        distance = EARTH_RADIUS_KM * c
        return distance

    @staticmethod
    def meters_to_degrees(meters: float, lat: float) -> Tuple[float, float]:
        return meters / 111000, meters / (111000 * math.cos(math.radians(lat)))

    @staticmethod
    def point_with_bearing(point: Point, distance: float, bearing: float) -> Point:
        bearing_rad = math.radians(bearing)
        lat_rad = math.radians(point.lat)
        lon_rad = math.radians(point.lon)

        new_lat_rad = math.asin(math.sin(lat_rad) * math.cos(distance / EARTH_RADIUS_M) +
                                math.cos(lat_rad) * math.sin(distance / EARTH_RADIUS_M) * math.cos(bearing_rad))

        new_lon_rad = lon_rad + math.atan2(math.sin(bearing_rad) * math.sin(distance / EARTH_RADIUS_M) * math.cos(lat_rad),
                                           math.cos(distance / EARTH_RADIUS_M) - math.sin(lat_rad) * math.sin(new_lat_rad))

        new_lat = math.degrees(new_lat_rad)
        new_lon = math.degrees(new_lon_rad)

        return Point(new_lat,new_lon)

    @staticmethod
    def calculate_bearing(point1: Point, point2: Point) -> float:
        lat1_rad = math.radians(point1.lat)
        lon1_rad = math.radians(point1.lon)
        lat2_rad = math.radians(point2.lat)
        lon2_rad = math.radians(point2.lon)

        dlon_rad = lon2_rad - lon1_rad

        y = math.sin(dlon_rad) * math.cos(lat2_rad)
        x = math.cos(lat1_rad) * math.sin(lat2_rad) - math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(dlon_rad)

        bearing_rad = math.atan2(y, x)
        bearing_deg = math.degrees(bearing_rad)
        bearing_deg = (bearing_deg + 360) % 360
        return bearing_deg
    
    @staticmethod
    def calculate_new_coordinates(lat: float, lon: float, bearing: float, distance: float) -> Tuple[float, float]:
        """
        Calculate new coordinates given a starting point, bearing, and distance.
        
        :param lat: Starting latitude in degrees
        :param lon: Starting longitude in degrees
        :param bearing: Bearing in degrees (0 is north, 90 is east, etc.)
        :param distance: Distance to travel in meters
        :return: Tuple of (new_latitude, new_longitude) in degrees
        """
        # Convert latitude, longitude, and bearing to radians
        lat_rad = math.radians(lat)
        lon_rad = math.radians(lon)
        bearing_rad = math.radians(bearing)

        # Calculate angular distance
        angular_distance = distance / EARTH_RADIUS_KM

        # Calculate new latitude
        new_lat_rad = math.asin(
            math.sin(lat_rad) * math.cos(angular_distance) +
            math.cos(lat_rad) * math.sin(angular_distance) * math.cos(bearing_rad)
        )

        # Calculate new longitude
        new_lon_rad = lon_rad + math.atan2(
            math.sin(bearing_rad) * math.sin(angular_distance) * math.cos(lat_rad),
            math.cos(angular_distance) - math.sin(lat_rad) * math.sin(new_lat_rad)
        )

        # Convert new latitude and longitude back to degrees
        new_lat = math.degrees(new_lat_rad)
        new_lon = math.degrees(new_lon_rad)

        # Normalize longitude to be within -180 to 180 degrees
        new_lon = (new_lon + 540) % 360 - 180

        return new_lat, new_lon
    
    @staticmethod
    def calc_yaw_diff(yaw1: float, yaw2: float) -> float:
        """
        Calculate the difference between two yaw angles.
        
        :param yaw1: First yaw angle in degrees
        :param yaw2: Second yaw angle in degrees
        :return: Difference between the two yaw angles in degrees
        """
        diff = yaw2 - yaw1
        while diff > 180:
            diff -= 360
        while diff < -180:
            diff += 360
        return diff
    
    @staticmethod
    def increase_distance(points: List[Point], middle_points: Set[Point], keep_every_nth: int) -> List[Point]:
        """
        Process a list of points, keeping all middle points, every nth point, and the last point.

        Args:
            points (List[Point]): The list of points to process.
            middle_points (Set[Point]): Set of points to always keep.
            keep_every_nth (int): Keep every nth point (n = keep_every_nth).

        Returns:
            List[Point]: The processed list of points.
        """
        if keep_every_nth < 1:
            raise ValueError("keep_every_nth must be a positive integer")

        result = []
        for i, point in enumerate(points):
            if point in middle_points or i % keep_every_nth == 0 or i == len(points) - 1:
                result.append(point)
        
        return result

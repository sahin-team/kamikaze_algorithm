import math
from typing import Tuple


class GeographicUtils:
    @staticmethod
    def haversine(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        R = 6371  # Radius of the Earth in km
        dlat = math.radians(lat2 - lat1)
        dlon = math.radians(lon2 - lon1)
        a = math.sin(dlat / 2) ** 2 + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(dlon / 2) ** 2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        distance = R * c
        return distance

    @staticmethod
    def meters_to_degrees(meters: float, lat: float) -> Tuple[float, float]:
        return meters / 111000, meters / (111000 * math.cos(math.radians(lat)))

    @staticmethod
    def point_with_bearing(lat: float, lon: float, distance: float, bearing: float) -> Tuple[float, float]:
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

    @staticmethod
    def calculate_bearing(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        lat1_rad = math.radians(lat1)
        lon1_rad = math.radians(lon1)
        lat2_rad = math.radians(lat2)
        lon2_rad = math.radians(lon2)

        dlon_rad = lon2_rad - lon1_rad

        y = math.sin(dlon_rad) * math.cos(lat2_rad)
        x = math.cos(lat1_rad) * math.sin(lat2_rad) - math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(dlon_rad)

        bearing_rad = math.atan2(y, x)
        bearing_deg = math.degrees(bearing_rad)
        bearing_deg = (bearing_deg + 360) % 360
        return bearing_deg

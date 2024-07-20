from typing import List, Tuple
from GeographicUtils import GeographicUtils
from Point import Point
from RedZone import RedZone


class ObstacleAvoidance:
    @staticmethod
    def is_point_in_red_zone(point: Point, red_zones: List[RedZone]) -> bool:
        for zone in red_zones:
            if GeographicUtils.haversine(point.lat, point.lon, zone.center.lat, zone.center.lon) <= (zone.radius + 30) / 1000:
                return True
        return False

    @staticmethod
    def path_is_clear_of_red_zones(path: List[Point], red_zones: List[RedZone]) -> bool:
        for point in path:
            if ObstacleAvoidance.is_point_in_red_zone(point, red_zones):
                return False
        return True
    
    @staticmethod
    def find_first_red_zone_point(path: List[Point], red_zones: List[RedZone]) -> Tuple[Point, RedZone]:
        for point in path:
            for zone in red_zones:
                if GeographicUtils.haversine(point.lat, point.lon, zone.center.lat, zone.center.lon) <= (zone.radius + 30) / 1000:
                    return point, zone
        return None, None
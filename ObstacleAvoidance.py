from typing import List, Tuple
from GeographicUtils import GeographicUtils
from Point import Point,RedZone

class ObstacleAvoidance:
    def __init__(self, red_zones: List[RedZone], boundaries: List[Point]):
        self.red_zones = red_zones
        self.boundaries = boundaries

    def is_point_in_red_zone(self, point: Point) -> bool:
        for zone in self.red_zones:
            if GeographicUtils.haversine(point, zone.center) <= (zone.radius + (zone.radius / 6)) / 1000:
                return True
        return False

    def path_is_clear_of_red_zones(self, path: List[Point]) -> bool:
        for point in path:
            if self.is_point_in_red_zone(point):
                return False
        return True
    
    def find_first_red_zone_point(self, path: List[Point]) -> Tuple[Point, RedZone]:
        for point in path:
            for zone in self.red_zones:
                if GeographicUtils.haversine(point, zone.center) <= (zone.radius + (zone.radius / 6)) / 1000:
                    return point, zone
        return None, None
    
    def is_point_in_boundaries(self, point: Point) -> bool:
        min_lat = min(boundary.lat for boundary in self.boundaries)
        max_lat = max(boundary.lat for boundary in self.boundaries)
        min_lon = min(boundary.lon for boundary in self.boundaries)
        max_lon = max(boundary.lon for boundary in self.boundaries)
        
        return min_lat <= point.lat <= max_lat and min_lon <= point.lon <= max_lon
    
    def is_point_valid(self, point: Point) -> bool:
        return not self.is_point_in_red_zone(point) and self.is_point_in_boundaries(point)

    def is_path_valid(self, path: List[Point]) -> bool:
        for point in path:
            if not self.is_point_valid(point):
                return False
        return True
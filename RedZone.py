from Point import Point


class RedZone:
    def __init__(self, id: int, lat: float, lon: float, radius: float):
        self.id = id
        self.center = Point(lat, lon)
        self.radius = radius
        
    def print_zone(self):
        print(f"ID: {self.id} - Center: {self.center.to_tuple()} - Radius: {self.radius}")

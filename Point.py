from typing import Tuple


class Point:
    def __init__(self, lat: float, lon: float):
        self.lat = lat
        self.lon = lon
    
    def to_tuple(self) -> Tuple[float, float]:
        return (self.lat, self.lon)
    
    def print_point(self):
        print(f"lat: {self.lat} - lon: {self.lon}")
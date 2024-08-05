from dataclasses import dataclass

@dataclass
class Point:
    lat: float = 0.0
    lon: float = 0.0

@dataclass
class RedZone:
    id: int
    center: Point
    radius: float

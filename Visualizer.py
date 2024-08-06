from typing import List
from matplotlib import pyplot as plt
from matplotlib.patches import Circle

from GeographicUtils import GeographicUtils
from Point import Point,RedZone

class Visualizer:
    @staticmethod
    def plot_red_zones(ax, red_zones: List[RedZone]):
        for zone in red_zones:
            lat, lon, radius = zone.center.lat, zone.center.lon, zone.radius
            radius_lat, radius_lon = GeographicUtils.meters_to_degrees(radius, lat)
            circle = Circle((lon, lat), radius_lon, color='red', alpha=0.5)
            ax.add_patch(circle)
    
    @staticmethod
    def plot_points(ax, start: Point, goal: Point, path: List[Point]):
        ax.plot(start.lon, start.lat, marker='o', markersize=10, color='blue', label='Drone')
        ax.plot(goal.lon, goal.lat, marker='x', markersize=10, color='green', label='Target')
        
        path_lons = [point.lon for point in path]
        path_lats = [point.lat for point in path]
        ax.plot(path_lons, path_lats, marker='o', color='orange', label='Path')
    
    @staticmethod
    def plot_middle_points(ax, middle_points: List[Point]):
        middle_lons = [point.lon for point in middle_points]
        middle_lats = [point.lat for point in middle_points]
        ax.plot(middle_lons, middle_lats, marker='o', color='purple', label='Middle Points')
    
    @staticmethod
    def set_plot_bounds(ax, red_zones: List[RedZone]):
        min_lat = min(zone.center.lat - GeographicUtils.meters_to_degrees(zone.radius, zone.center.lat)[0] for zone in red_zones)
        max_lat = max(zone.center.lat + GeographicUtils.meters_to_degrees(zone.radius, zone.center.lat)[0] for zone in red_zones)
        min_lon = min(zone.center.lon - GeographicUtils.meters_to_degrees(zone.radius, zone.center.lat)[1] for zone in red_zones)
        max_lon = max(zone.center.lon + GeographicUtils.meters_to_degrees(zone.radius, zone.center.lat)[1] for zone in red_zones)

        buffer_factor = 0.8
        lat_buffer = buffer_factor * (max_lat - min_lat)
        lon_buffer = buffer_factor * (max_lon - min_lon)
        ax.set_xlim(min_lon - lon_buffer, max_lon + lon_buffer)
        ax.set_ylim(min_lat - lat_buffer, max_lat + lat_buffer)
    
    @staticmethod
    def plot_path(path: List[Point], red_zones: List[RedZone], start: Point, goal: Point, middle_points: List[Point] = None,boundary_points:List[Point] = None):
        fig, ax = plt.subplots()
        
        Visualizer.plot_red_zones(ax, red_zones)
        Visualizer.plot_points(ax, start, goal, path)
        if middle_points:
            Visualizer.plot_middle_points(ax, middle_points)
        if boundary_points:
            Visualizer.plot_middle_points(ax, boundary_points)
        Visualizer.set_plot_bounds(ax, red_zones)

        ax.set_xlabel("lon")
        ax.set_ylabel("lat")
        ax.set_title("Red Zones with Drone and Target")
        ax.legend()

        plt.gca().set_aspect('equal', adjustable='box')
        plt.grid(True)
        plt.show()
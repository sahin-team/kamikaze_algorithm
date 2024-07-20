from DroneNavigator import DroneNavigator
from Point import Point
from RedZone import RedZone


def main():
    drone_start = Point(40.23221, 29.00999)
    # drone_start = Point(40.22999, 29.00828)
    
    
    qr_code_goal = Point(40.23008, 29.00499)

    red_zones = [
        RedZone(0, 40.23260922, 29.00573015, 50),
        RedZone(1, 40.23351019, 28.99976492, 50),
        RedZone(2, 40.23105297, 29.00744677, 75),
        RedZone(3, 40.23090554, 29.00221109, 170)
    ]
    
    navigator = DroneNavigator(drone_start, qr_code_goal, red_zones)
    final_path = navigator.navigate()
    

if __name__ == "__main__":
    main()
    
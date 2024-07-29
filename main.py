from DroneNavigator import DroneNavigator
from Point import Point
from RedZone import RedZone


def main():
    # drone_start = Point(40.22999, 29.00828) 
    # drone_start = Point(40.22997, 28.99953)
    # drone_start = Point(40.23313, 29.00429) #line
    # drone_start = Point(40.23107, 29.00900)
    drone_start = Point(40.23221, 29.00999)
    # drone_start = Point(40.23392, 28.99619)
    # drone_start = Point(40.22844,28.99969)  #two points
    # drone_start = Point(40.23187,28.99960) #problem  #two points
    
    
    qr_code_goal = Point(40.23008, 29.00499)
    # qr_code_goal = Point(40.23457, 29.00155)
    # qr_code_goal = Point(40.23473,29.00398)

    red_zones = [
        RedZone(0, 40.23260922, 29.00573015, 50),
        RedZone(1, 40.23351019, 28.99976492, 50),
        RedZone(2, 40.23105297, 29.00744677, 75),
        RedZone(3, 40.23090554, 29.00221109, 170)
    ]
    
    current_yaw = 90
    
    navigator = DroneNavigator(drone_start, qr_code_goal, red_zones, current_yaw)
    final_path = navigator.navigate()
    

if __name__ == "__main__":
    main()
    

import time
import numpy as np

# Libraries crazyflie
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper

# Libraries personnelles
from drone import Drone
from arena import Arena, Direction
from crossing_middle_zone import obstacle_detection
from search_platform import edge_detection

class states():
    edge_detected = False

def main_back_home(drone:Drone, going_home_line, height):

    THRESHOLD_LANDING_Y = 0.05
    THRESHOLD_LANDING_X = 0.6
    SPEED_FORWARD = 0.5
    SPEED_LATERAL = 0.5

    position_estimate = [0, 0]
    arrived = False
    on_platform = False

    position_estimate[0] = drone.get_log('stateEstimate.x')
    position_estimate[1] = drone.get_log('stateEstimate.y')
    print(f"x = {position_estimate[0]:.3f}  y = {position_estimate[1]:.3f}")

    speed_x, speed_y = obstacle_detection(drone, going_home_line, forward_speed=SPEED_FORWARD, lateral_come_back_speed=SPEED_LATERAL, direction=Direction.BACKWARD)
    edge_detected = edge_detection(drone, fly_height=height, threshold=drone.TRESHOLD_UP)
    if position_estimate[0] < Arena.LENGTH/2 and edge_detected:
        print("edge detection authorized")
        states.edge_detected = True
        arrived = True
        on_platform = True
        drone.stop()
    # si la position en X est validée, stop le drone en X (pourrait avoir évité un obstacle et doit revenir)
    if position_estimate[0] <= -THRESHOLD_LANDING_X:
        speed_x = 0
        # set la direction du drone pour le landing
        if speed_y > 0:
            drone.direction = Direction.LEFT
        else:
            drone.direction = Direction.RIGHT
    else:
        drone.direction = Direction.BACKWARD
  
    # si la positionen X et Y est validée
    if (position_estimate[0] <= -THRESHOLD_LANDING_X) and (abs(position_estimate[1]) <= THRESHOLD_LANDING_Y) and not states.edge_detected:
        arrived = True
        on_platform = False
        drone.stop()

    if not arrived and not on_platform:
        drone.start_linear_motion(speed_x, speed_y, 0)
        
    return arrived, on_platform

def main_back_home_simple(drone:Drone, going_home_line, height):

    position_estimate = [0, 0]
    arrived = False
    THRESHOLD_LANDING = 0.05

    position_estimate[0] = drone.get_log('stateEstimate.x')
    position_estimate[1] = drone.get_log('stateEstimate.y')
    print(f"x = {position_estimate[0]:.3f}  y = {position_estimate[1]:.3f}")

    speed_x, speed_y = obstacle_detection(drone, going_home_line, forward_speed=0.3, lateral_come_back_speed=0.3, direction=Direction.BACKWARD)
    edge_detected = edge_detection(drone, fly_height=height, threshold=drone.TRESHOLD_UP)
    if position_estimate[0] < Arena.LENGTH/2 and edge_detected:
        states.test = True
    # si la position en X est validée, stop le drone en X (pourrait avoir évité un obstacle et doit revenir)
    if position_estimate[0] <= THRESHOLD_LANDING:
        speed_x = 0
    # si la positionen X et Y est validée
    if (position_estimate[0] <= THRESHOLD_LANDING) and (abs(position_estimate[1]) <= THRESHOLD_LANDING):
        arrived = True
    if not arrived:
        drone.start_linear_motion(speed_x, speed_y, 0)
        
    return arrived

    # pass
if __name__ == '__main__':
    # initalise les drivers low level, obligatoire
    cflib.crtp.init_drivers()
    # identifiant radio du drone
    URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E702')

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        # crée un drone (hérite de motion commander)
        drone = Drone(scf, default_height=0.2)

        #le drone suit la ligne au centre de l'arène
        going_home_line = 0
        drone.start_logs()

        drone.take_off()
        # drone.go_to(1.5, 1)
        drone.position_x_offset = 4.250
        drone.position_y_offset = 0.181

        arrived_home = False
        while(not arrived_home):
            drone.update_slam()
            drone.stop_by_hand()
            arrived_home = main_back_home(drone, going_home_line, height=0.2)
            time.sleep(0.1)

        drone.land()
        print(f'the drone detected an edge {states.test}')
        drone.slam.hold()

        drone.stop_logs(save=False)
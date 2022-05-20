
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

def main_back_home(drone:Drone, going_home_line):
    range_sensors = np.empty(5)
    position_estimate = [0, 0]
    arrived = False
    speed_x, speed_y = obstacle_detection(drone, going_home_line, Direction.BACKWARD)

    position_estimate[0] = drone.get_log('stateEstimate.x')
    position_estimate[1] = drone.get_log('stateEstimate.y')
    threshold_landing = 0.05

    if (abs(position_estimate[0] - 0) <= threshold_landing) and (abs(position_estimate[1] - 0) <= threshold_landing):
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
        drone.go_to(1.5, 1)

        arrived_home = False
        while(not arrived_home):
            drone.update_slam()
            drone.stop_by_hand()
            arrived_home = main_back_home(drone, going_home_line)

        drone.land()
        drone.slam.hold()

        drone.stop_logs(save=False)
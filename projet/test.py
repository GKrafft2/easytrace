# Libraries python
import time
from datetime import timedelta
from enum import Enum

# Libraries crazyflie
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper

# Libraries personnelles
from drone import Drone

# parties du projet
from crossing_middle_zone import crossing_middle_zone, states as states_crossing
from landing_procedure import go_to_P
from search_platform import edge_detection
from search_platform import search_platform, distance_detection
from back_home import main_back_home
from landing_procedure import landing_procedure
from arena import Arena, Direction


if __name__ == '__main__':

    # initalise les drivers low level, obligatoire
    cflib.crtp.init_drivers()
    # identifiant radio du drone
    URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E702')

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:

        # crée un drone (hérite de motion commander)
        drone = Drone(scf, default_height=0.2)
        drone.start_logs()

        # chronomètre le temps de vol
        start_time = time.perf_counter()
        
        drone.take_off()
        drone.start_linear_motion(0.5, 0, 0)
        edge_detected = False
        while(not edge_detected):
            time.sleep(0.1)
            edge_detected = edge_detection(drone, 0.2, drone.TRESHOLD_UP)
        position_start = drone.get_log('stateEstimate.x')
        # time.sleep(2)
        # drone.stop_brutal(2.2)
        # time.sleep(1)
        drone.start_linear_motion(0.1, 0, 0)
        edge_detected = False
        while(not edge_detected):
            time.sleep(0.1)
            edge_detected = edge_detection(drone, 0.2, drone.TRESHOLD_UP)
        position_end = drone.get_log('stateEstimate.x')
        drone.start_linear_motion(0.2, 0, 0)
        time.sleep(1)
        drone.stop()
        go_to_P(drone, position_start + (position_end-position_start)/2 - 0.09, 0)

        # time.sleep(1)

        drone.land()
        # reached = False
        # while(not reached):
        #     reached = distance_detection(drone, 0.2, 0, Direction.FORWARD)
        #     if not reached:
        #         drone.start_linear_motion(0.2, 0, 0)
        #     print(drone.get_log('stateEstimate.x'))
        #     time.sleep(0.1)

        # drone.go_to(z=0.003, velocity=0.1)
        # drone.land()
        # time.sleep(1)
        # drone.take_off()
        # reached = False
        # while(not reached):
        #     reached = distance_detection(drone, 0.2, 0, Direction.FORWARD)
        #     if not reached:
        #         drone.start_linear_motion(0.2, 0, 0)
        #     print(drone.get_log('stateEstimate.x'))
        #     time.sleep(0.1)
        # drone.land(velocity=0.05)

        # drone.stop_logs()

        # # affiche le temps de vol
        # end_time = time.perf_counter()
        # print(f'Total fly time : {timedelta(seconds=round(end_time-start_time, 0))} \n')    

        # drone.slam.hold()



      
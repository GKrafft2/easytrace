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
from search_platform import search_platform, states as state_search_platform
from back_home import main_back_home
from landing_procedure import landing_procedure
from arena import Arena, Platform

class States(Enum):
    START = -1
    CROSSING_MIDDLE_ZONE = 0
    SEARCHING_PLATFORM_P2 = 1
    LANDING_P2 = 2
    GOING_HOME = 3
    LANDING_P1 = 4
    SEARCHING_PLATFORM_P1 = 5
    END = 6


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

        # State machine
        state = States.START

        # state = States.GOING_HOME
        # drone.take_off(0.2)
        # time.sleep(1)
        # drone.position_x_offset = 3.609
        # drone.position_y_offset = 1.887

        while(state is not States.END):

            # fonction constemment évaluées
            drone.update_slam()

            if state == States.START:
                # print(" ===== STATE START =====")
                drone.take_off()
                #le drone suit la ligne au centre de l'arène
                central_line = - (Arena.ORIGIN_Y - Arena.WIDTH/2)
                # update de la direction par défault s'il y a un obstacle
                if central_line > 0:
                    states_crossing.default_direction = 1   # gauche
                else:
                    states_crossing.default_direction = -1  # droite
                state = States.CROSSING_MIDDLE_ZONE
                print(" ===== STATE CROSSING =====")

            if state == States.CROSSING_MIDDLE_ZONE:
                
                # fonction continue
                crossed_middle_zone = crossing_middle_zone(drone, central_line)
                if crossed_middle_zone:
                    state = States.SEARCHING_PLATFORM_P2
                    edge_detected = False
                    # time.sleep(2)
                    print(" ===== STATE SEARCH PLATFORM =====")

            if state == States.SEARCHING_PLATFORM_P2:
                
                # fonction continue
                edge_detected = search_platform(drone, Arena.WIDTH, Platform.SIZE, Arena.LIM_WEST - drone.get_log('stateEstimate.y'), height=0.2)
                if edge_detected:
                    state = States.LANDING_P2
                    print(" ===== STATE LANDING P2 =====")
            if state == States.LANDING_P2:
                
                # drone.stop()
                # fonction bloquante
                landing_procedure(drone, drone.direction, height=0.2)
                drone.slam.save_img()
                state = States.GOING_HOME
                drone.take_off()
                print(" ===== STATE GOING HOME =====")

            if state == States.GOING_HOME:

                # fonction continue
                going_home_line = 0
                arrived_home, on_platform = main_back_home(drone, going_home_line, height=0.2)
                if arrived_home and on_platform:
                    print("Landing mode direct")
                    state = States.LANDING_P1
                if arrived_home and not on_platform:
                    print("Landing mode mini-search")
                    state = States.SEARCHING_PLATFORM_P1
                    # reset param from platforme P2
                    state_search_platform.next_segment = True
                    state_search_platform.segment = 0
                
            if state == States.LANDING_P1:
                # fonction bloquante
                print("Final landing")
                landing_procedure(drone, drone.direction, height=0.2)
                state = States.END

            if state == States.SEARCHING_PLATFORM_P1:
                # fonction continue
                print("search platforme P1")
                edge_detected = search_platform(drone, 1.25, 0.23, 0.5 ,height=0.2)
                if edge_detected:
                    print("slaut")
                    state = States.LANDING_P1


            # Délai important pour ne pas overflood les envois de données au drone
            time.sleep(0.1)
            drone.stop_by_hand()

        drone.stop_logs(save=False)

        # affiche le temps de vol
        end_time = time.perf_counter()
        print(f'Total fly time : {timedelta(seconds=round(end_time-start_time, 0))} \n')    

        drone.slam.hold()
        drone.slam.save_img()

        input("Press Enter to delete SLAM...")


      
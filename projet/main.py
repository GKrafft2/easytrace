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
from crossing_middle_zone import crossing_middle_zone
from search_platform import search_platform
from back_home import main_back_home
from landing_procedure import landing_procedure
from arena import Arena

class States(Enum):
    START = -1
    CROSSING_MIDDLE_ZONE = 0
    SEARCHING_PLATFORM = 1
    LANDING_P2 = 2
    GOING_HOME = 3
    LANDING_P1 = 4
    END = 5


if __name__ == '__main__':

    # initalise les drivers low level, obligatoire
    cflib.crtp.init_drivers()
    # identifiant radio du drone
    URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E702')

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:

        # crée un drone (hérite de motion commander)
        drone = Drone(scf, default_height=0.4)
        drone.start_logs()

        # chronomètre le temps de vol
        start_time = time.perf_counter()

        # State machine
        state = States.START
        while(state is not States.END):

            # fonction constemment évaluées
            drone.update_slam()
            drone.stop_by_hand()

            if state == States.START:

                drone.take_off()
                #le drone suit la ligne au centre de l'arène
                central_line = - (Arena.ORIGIN_Y - Arena.WIDTH/2)
                # update de la direction par défault s'il y a un obstacle
                if central_line > 0:
                    drone.default_direction = 1   # gauche
                else:
                    drone.default_direction = -1  # droite
                state = States.CROSSING_MIDDLE_ZONE

            if state == States.CROSSING_MIDDLE_ZONE:

                # fonction continue
                crossed_middle_zone = crossing_middle_zone(drone, central_line)
                if crossed_middle_zone:
                    state = States.SEARCHING_PLATFORM
                    drone.next_segment = True
                    drone.segment = 0

            if state == States.SEARCHING_PLATFORM:

                # fonction continue
                edge_detected = search_platform(drone, height=0.2)
                if edge_detected:
                    state = States.LANDING_P2

            if state == States.LANDING_P2:

                # fonction bloquante
                landing_procedure(drone, drone.direction, height=0.2)
                state = States.GOING_HOME

            if state == States.GOING_HOME:

                # fonction continue
                pass
                # state = States.LANDING_P1

            if state == States.LANDING_P1:
                
                # fonction bloquante
                landing_procedure(drone, drone.direction, height=0.2, search_first_edge=True)
        
            # Délai important pour ne pas overflood les envois de données au drone
            time.sleep(0.1)

        drone.stop_logs()

        # affiche le temps de vol
        end_time = time.perf_counter()
        print(f'Total fly time : {timedelta(seconds=round(end_time-start_time, 0))} \n')    

        drone.slam.hold()



      
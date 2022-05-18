# Libraries python
import time
import datetime as dt
from threading import Event
import numpy as np
from enum import Enum

# Libraries crazyflie
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper

# Libraries personnelles
from drone import Drone

# parties du projet
from p_crossing import main_crossing
from p_search_platform import main_search_platform
from p_back_home import main_back_home
from arena import Arena, Direction
import p_crossing as crossing
from p_land_platform import landing_procedure

class States(Enum):
    START = -1
    CROSSING = 0
    SEARCHING_PLATFORM = 1
    LANDING = 2
    GOING_HOME = 3
    END = 4


if __name__ == '__main__':

    # initalise les drivers low level, obligatoire
    cflib.crtp.init_drivers()
    # identifiant radio du drone
    URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E702')

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:

        # crée un drone (hérite de motion commander)
        drone = Drone(scf, default_height=0.4)
        drone.start_logs()

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
                    drone.default_direction = 1
                else:
                    drone.default_direction = -1

                state = States.CROSSING

            if state == States.CROSSING:
                arrival = crossing.start_zone_2_check(drone)
                if not arrival:
                    speed_x, speed_y = crossing.avoid(drone, central_line, Direction.FORWARD)
                    drone.start_linear_motion(speed_x, speed_y, 0)
                else:
                    state = States.SEARCHING_PLATFORM

            if state == States.SEARCHING_PLATFORM:
                # fonction bloquante
                main_search_platform(drone)

                state = States.LANDING

            if state == States.LANDING:
                # fonction bloquante
                landing_procedure(drone, drone.direction)

            if state == States.GOING_HOME:
                pass
                # state = States.LANDINGW
        
            time.sleep(0.1)

        drone.stop_logs()



      
# python libraries
import time
from datetime import timedelta
from enum import Enum

# Crazyflie Libraries
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper

# Personal Libraries
from drone import Drone

# parts of the project
from crossing_middle_zone import crossing_middle_zone,go_to_middle, states as states_crossing
from search_platform import search_platform, states as state_search_platform
from back_home import main_back_home
from landing_procedure import landing_procedure
from arena import Arena, Platform

class States(Enum):
    START = -1
    CROSSING_MIDDLE_ZONE = 0
    SEARCHING_PLATFORM_P2 = 1
    LANDING_P2 = 2
    TACK_OFF_ALIGN = 3
    GOING_HOME = 4
    LANDING_P1 = 5
    SEARCHING_PLATFORM_P1 = 6
    END = 7


if __name__ == '__main__':

    # initialize low level drivers, mandatory    cflib.crtp.init_drivers()
    # drone radio identifier
    URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E702')
    offset = Arena.ORIGIN_Y
    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:

        # create a drone (inherit from motion commander)
        drone = Drone(scf, default_height=0.2)
        drone.start_logs()

        # times flight time
        start_time = time.perf_counter()

        # State machine
        state = States.START

        while(state is not States.END):

            # functions are constantly evaluated every 0.1 milliseconds
            drone.update_slam()

            if state == States.START:
                drone.take_off()
                #le drone suit la ligne au centre de l'arène
                central_line = 0
                # central_line = -(Arena.ORIGIN_Y - offset)
                # update the default direction if there is an obstacle
                state = States.CROSSING_MIDDLE_ZONE

            if state == States.CROSSING_MIDDLE_ZONE:
                crossed_middle_zone = crossing_middle_zone(drone, central_line)
                if crossed_middle_zone:
                    state = States.SEARCHING_PLATFORM_P2
                    edge_detected = False

            if state == States.SEARCHING_PLATFORM_P2:
                edge_detected = search_platform(drone, Arena.WIDTH, Platform.SIZE, Arena.LIM_WEST - drone.get_log('stateEstimate.y'), height=0.2)
                if edge_detected:
                    state = States.LANDING_P2

            if state == States.LANDING_P2:
                # blocking function
                landing_procedure(drone, drone.direction, height=0.2)
                drone.slam.save_img()
                state = States.TACK_OFF_ALIGN
                drone.take_off()

            if state == States.TACK_OFF_ALIGN:
                print('state tack_off_align')
                crossed_middle_zone = go_to_middle(drone, drone.get_log('stateEstimate.x'),drone.get_log('stateEstimate.y'))

                if crossed_middle_zone:
                    state = States.GOING_HOME

            if state == States.GOING_HOME:
                print('state going home')
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
                print("search platforme P1")
                edge_detected = search_platform(drone, 1.25, 0.23, 0.5 ,height=0.2)
                if edge_detected:
                    print("slaut")
                    state = States.LANDING_P1


            # Significant delay so as not to overflood sending data to the drone
            time.sleep(0.1)
            drone.stop_by_hand()

        drone.stop_logs(save=False)

        # show flight time
        end_time = time.perf_counter()
        print(f'Total fly time : {timedelta(seconds=round(end_time-start_time, 0))} \n')    

        drone.slam.hold()
        drone.slam.save_img()

        input("Press Enter to delete SLAM...")


      
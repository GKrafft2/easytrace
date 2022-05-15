import logging
import sys
import os

# Libraries python
import time
import datetime as dt
from threading import Event
import numpy as np

# Libraries crazyflie
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper

# Libraries personnelles
from Easytrace_drone import Easytrace
from zone_test_slam import Slam
from arena import arena_dim, arena_limits, platform


if __name__ == '__main__':
    # initalise les drivers low level, obligatoire
    cflib.crtp.init_drivers()
    # identifiant radio du drone
    URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E702')

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        # crée un drone (hérite de motion commander)
        drone = Easytrace(scf, default_height=0.2)
        slam = Slam()

        drone.start_logs()
        drone.take_off()

        fly = True

        # obstacle and slam test
        # while fly:
        #     drone.refresh_logs()
        #     slam.slam_update(drone.range_sensors,drone.position_estimate)
        #     drone.crossing_avoid(front=True)
        #     if drone.position_estimate[0] > 3:
        #         fly = False


        # spiral search test
        #while fly:
         #   slam.slam_update(drone.range_sensors, drone.position_estimate)
        drone.spiral_search()


        print("land")
        drone.land()

        slam.slam_hold()
        drone.stop_logs(save=False)
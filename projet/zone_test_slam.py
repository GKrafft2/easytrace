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
import matplotlib.pyplot as plt

# Libraries personnelles
from Easytrace_drone import Easytrace
from arena import arena_dim, arena_limits, platform

map_shape = (500, 300)
map = np.zeros(map_shape)

offset_y = np.floor(map_shape[1]/2)

plt.ion()
fig = plt.figure()

def fly_while_slam(drone:Easytrace):
    fly = True
    range_sensors = np.empty(5)
    position_estimate = [offset_y, 0]


    #while fly:
    for i in range(300):

        range_sensors[0] = drone.get_log('range.front')
        range_sensors[1] = drone.get_log('range.back')
        range_sensors[2] = drone.get_log('range.left')
        range_sensors[3] = drone.get_log('range.right')
        range_sensors[4] = drone.get_log('range.up')

        position_estimate[0] = np.floor(drone.get_log('stateEstimate.x')*100)
        position_estimate[1] = offset_y + np.floor(drone.get_log('stateEstimate.y')*100)

        map[int(position_estimate[0])][int(position_estimate[1])] = 1

        if i%10 == 0:
            plt.imshow(map)
            fig.canvas.draw()
            fig.canvas.flush_events()

        drone.start_linear_motion(0.2, 0, 0)




if __name__ == '__main__':
    # initalise les drivers low level, obligatoire
    cflib.crtp.init_drivers()
    # identifiant radio du drone
    URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E702')

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        # crée un drone (hérite de motion commander)
        drone = Easytrace(scf, default_height=0.2)

        drone.start_logs()

        drone.take_off()

        fly_while_slam(drone)

        print("land")
        drone.land()

        drone.stop_logs(save=False)
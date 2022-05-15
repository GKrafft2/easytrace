import logging
import sys
import os

# Libraries python
import time
import datetime as dt
from threading import Event
import numpy as np
import cv2

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
# creer un offset_x pour le projet final


def fly_while_slam(drone:Easytrace):
    fly = True
    range_sensors = np.empty(5)
    position_estimate = [offset_y, 0]

    thresh_obstcl = 50
    blue = 64
    red = 255

    #while fly:
    for i in range(3000):

        range_sensors[0] = np.floor(drone.get_log('range.front')/10)
        range_sensors[1] = np.floor(drone.get_log('range.back')/10)
        range_sensors[2] = np.floor(drone.get_log('range.left')/10)
        range_sensors[3] = np.floor(drone.get_log('range.right')/10)
        range_sensors[4] = drone.get_log('range.up')

        position_estimate[0] = np.floor(drone.get_log('stateEstimate.x')*100)
        position_estimate[1] = offset_y + np.floor(drone.get_log('stateEstimate.y')*100)

        print(i)
        print(position_estimate)

        # position mapping TODO : add limit cases
        map[int(position_estimate[0])][int(position_estimate[1])] = blue

        # obstcale mapping
        if position_estimate[0] + range_sensors[0] < map.shape[0]:
            if range_sensors[0] < thresh_obstcl:
                map[int(position_estimate[0] + range_sensors[0])][int(position_estimate[1])] = red  # front
        if position_estimate[0] - range_sensors[1] > 0:
            if range_sensors[1] < thresh_obstcl:
                map[int(position_estimate[0] - range_sensors[1])][int(position_estimate[1])] = red  # back
        if position_estimate[1] + range_sensors[2] < map.shape[1]:
            if range_sensors[2] < thresh_obstcl:
                map[int(position_estimate[0])][int(position_estimate[1] + range_sensors[2])] = red  # left
        if position_estimate[1] - range_sensors[3] > 0:
            if range_sensors[3] < thresh_obstcl:
                map[int(position_estimate[0])][int(position_estimate[1] - range_sensors[3])] = red  # right

        graymap = map.astype('uint8')
        colormap = cv2.applyColorMap(graymap, cv2.COLORMAP_JET)
        imS = cv2.resize(colormap, (map.shape[1], map.shape[0]))
        cv2.imshow('slam', imS)
        cv2.waitKey(1)

        # L shaped path for testing
        if i <= 1000 :
            drone.start_linear_motion(0.2, 0, 0)
        elif i > 1000 and i <= 1500 :
            drone.start_linear_motion(0, 0.2, 0)
        elif i > 1500 and i <= 2000 :
            drone.start_linear_motion(0, -0.2, 0)
        else :
            drone.start_linear_motion(-0.2, 0, 0)

    drone.land()
    # le drone se pose et l affichage du SLAM reste actif
    cv2.waitKey(0)




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
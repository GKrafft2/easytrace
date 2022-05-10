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
from arena import arena_dim, arena_limits, platform


def fly_while_avoid(drone:Easytrace):
    fly = True

    AVOID_DIST_LAT = 150  #mm
    AVOID_DIST_FRONT = 400  # mm
    AVOID_SPEED_LAT = 0.2
    AVOID_SPEED_FRONT = 0.5
    FORWARD_SPEED = 0.3

    # cree un array pour mettre les 5 variables de sensor
    # range [front,back,left,right,up]
    range_sensors = np.empty(5)

    position_estimate = [0, 0]

    drone.speed_x_cmd = 0
    drone.speed_y_cmd = 0

    prev_speed = 69
    speed_y = 0
    speed_y_lat = 0
    speed_y_front = 0

    right = -1
    left = 1

    avoid_dir = right #drone va commencer par éviter les obstacles par la droite

    while fly:

        range_sensors[0] = drone.get_log('range.front')
        range_sensors[1] = drone.get_log('range.back')
        range_sensors[2] = drone.get_log('range.left')
        range_sensors[3] = drone.get_log('range.right')
        range_sensors[4] = drone.get_log('range.up')

        position_estimate[0] = drone.get_log('stateEstimate.x')
        position_estimate[1] = drone.get_log('stateEstimate.y')

        print(range_sensors)

        # -- évitement latéral --

        if range_sensors[2] < AVOID_DIST_LAT:  # obstacle détecté à gauche
            #print('obstacle a gauche')
            speed_y_lat = -AVOID_SPEED_LAT

        elif range_sensors[3] < AVOID_DIST_LAT:  # obstacle détecté à droite
            #print('obstacle a droite')
            speed_y_lat = AVOID_SPEED_LAT

        else:
            #print('aucun obstcale lateral')
            speed_y_lat = 0

        # --------------------------

        # -- évitement frontal -----

        if range_sensors[0] < AVOID_DIST_FRONT:
            print("obstacle frontal !")

            if position_estimate[1] > 0.5:  # trop a gauche doit éviter par la droite
                #print('obstacle devant drone dans la partie gauche doite aller a droite')
                avoid_dir = right

            elif position_estimate[1] < -0.5:  # trop a droite doit éviter par la gauche
                #print('obstacle devant drone dans la partie droite doit aller a gauche ')
                avoid_dir = left

            speed_y_front = avoid_dir * AVOID_SPEED_FRONT
        else:
            #print('aucun obstcale frontal')
            speed_y_front = 0

        # --------------------------
        speed_y = (speed_y_lat + speed_y_front)

        if speed_y != 0:
            correction = FORWARD_SPEED/2  # correction pour aller plus lentement quand il y a des obstacles
        else:
            correction = 0

        if prev_speed != speed_y: # met a jour la commande que si elle est différente de la précédente
            print(f'maj de la vitesse avec une speed y = {speed_y}')
            drone.start_linear_motion(FORWARD_SPEED-correction, speed_y, 0)
            time.sleep(0.15) # set la refresh rate de l'évitement / laisset let temps au drone d effectuer le changement de direction

        prev_speed = speed_y

        #print(f'speed y = {speed_y}')

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

        fly_while_avoid(drone)

        print("land")
        drone.land()

        drone.stop_logs(save=False)
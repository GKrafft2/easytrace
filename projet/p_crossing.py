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
from drone import Drone

time1 = 0

def avoid(drone:Drone):

    global time1
    AVOID_DIST_LAT = 150  #mm
    AVOID_DIST_FRONT = 400  # mm
    AVOID_SPEED_LAT = 0.3
    AVOID_SPEED_FRONT = 0.3
    FORWARD_SPEED = 0.2

    # cree un array pour mettre les 5 variables de sensor
    # range [front,back,left,right,up]
    range_sensors = np.empty(5)

    position_estimate = [0, 0]

    position_direction = 0
    POSITION_DIRECTION_THRESH = 0.05

    prev_speed = 69
    correction = 0
    speed_y = 0
    speed_y_lat = 0
    speed_y_front = 0

    RIGHT = -1
    LEFT = 1

    avoid_dir = RIGHT #drone va commencer par éviter les obstacles par la droite

    range_sensors[0] = drone.get_log('range.front')
    range_sensors[1] = drone.get_log('range.back')
    range_sensors[2] = drone.get_log('range.left')
    range_sensors[3] = drone.get_log('range.right')
    range_sensors[4] = drone.get_log('range.up')

    position_estimate[0] = drone.get_log('stateEstimate.x')
    position_estimate[1] = drone.get_log('stateEstimate.y')

    # print(range_sensors)

    # ====== évitement latéral =======
    if range_sensors[2] < AVOID_DIST_LAT:  # obstacle détecté à gauche
        drone.obstacle_detected = True
        speed_y_lat = -AVOID_SPEED_LAT
    elif range_sensors[3] < AVOID_DIST_LAT:  # obstacle détecté à droite
        drone.obstacle_detected = True
        speed_y_lat = AVOID_SPEED_LAT
    else:
        speed_y_lat = 0


    # ====== évitement frontal ========
    if range_sensors[0] < AVOID_DIST_FRONT:
        drone.obstacle_detected = True
        if position_estimate[1] > position_direction + 0.1:  # trop a gauche doit éviter par la droite
            avoid_dir = RIGHT
        elif position_estimate[1] < -position_direction - 0.1:  # trop a droite doit éviter par la gauche
            avoid_dir = LEFT

        speed_y_front = avoid_dir * AVOID_SPEED_FRONT
    else:
        speed_y_front = 0


    # ====== update la vitesse en y ======
    speed_y = (speed_y_lat + speed_y_front)

    if speed_y != 0:
        correction = FORWARD_SPEED/2  # correction pour aller plus lentement quand il y a des obstacles
    
    # ====== revient à la ligne directrice si pas d'obstacle ===============
    # if drone.obstacle_detected == False:
    # correction = 0
    # if position_estimate[1] > position_direction + POSITION_DIRECTION_THRESH:
    #     speed_y = -AVOID_SPEED_LAT/2
    # elif position_estimate[1] < position_direction - POSITION_DIRECTION_THRESH:
    #     speed_y = AVOID_SPEED_LAT/2
    # else:
    #     speed_y = 0

    # ===== Délai avant la déclaration de fin d'obstacle pour revenir sur la ligne de direction
    # if drone.obstacle_detected == True:
    #     time1 = time.time_ns()
    # elif time.time_ns() - time1 > 5*1e9:
    #     drone.obstacle_detected == False
    #     print("plus d'obstacle")
        
    speed_x = FORWARD_SPEED-correction
    
    return speed_x, speed_y

def main_crossing(drone:Drone):
    fly = True

    while(fly):        
        drone.stop_by_hand()
        limits = drone.limits_check()
        if not limits:
            speed_x, speed_y = avoid(drone)
            drone.start_linear_motion(speed_x, speed_y, 0)
            # drone.slam.slam_update() # quoi mettre en paramètre ?
        else:
            fly = False
        time.sleep(0.1)



if __name__ == '__main__':
    # initalise les drivers low level, obligatoire
    cflib.crtp.init_drivers()
    # identifiant radio du drone
    URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E702')

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        # crée un drone (hérite de motion commander)
        drone = Drone(scf, default_height=0.2)

        drone.start_logs()

        drone.take_off()

        main_crossing(drone)

        drone.land()

        drone.stop_logs(save=False)
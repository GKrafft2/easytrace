from enum import Enum
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
from pyparsing import Forward

# Libraries personnelles
from drone import Drone
from arena import arena

class Direction(Enum):
    FORWARD = 0
    RIGHT = 1
    LEFT = 2
    BACKWARD = 3


def avoid(drone:Drone, line_position, direction:Direction):
    """ avoid obstacle live
        Param: line_position the y line to follow from drone origin """

    AVOID_DIST_LAT = 150  #mm
    AVOID_DIST_FRONT = 400  # mm
    AVOID_SPEED_LAT = 0.5
    AVOID_SPEED_COME_BACK = 0.25
    AVOID_TIME_COME_BACK = 1 # secondes
    AVOID_SPEED_FRONT = 0.3
    FORWARD_SPEED = 0.2


    # cree un array pour mettre les 5 variables de sensor
    # range [front,back,left,right,up]
    range_sensors = np.empty(5)

    position_estimate = [0, 0]

    POSITION_DIRECTION_THRESH = 0.05

    correction = 0
    speed_east = 0
    speed_east_lat = 0
    speed_east_front = 0

    RIGHT = -1
    LEFT = 1

    avoid_dir = drone.default_direction #drone va commencer par éviter les obstacles par la droite

    # ajuste les capteurs en fonction de la direction de déplacement
    if direction == Direction.FORWARD: # référence
        range_sensors[0] = drone.get_log('range.front')
        range_sensors[1] = drone.get_log('range.back')
        range_sensors[2] = drone.get_log('range.left')
        range_sensors[3] = drone.get_log('range.right')
        position_estimate[0] = drone.get_log('stateEstimate.x')
        position_estimate[1] = drone.get_log('stateEstimate.y')
        RIGHT = -1
        LEFT = 1
    elif direction == Direction.LEFT:
        range_sensors[3] = drone.get_log('range.front')
        range_sensors[2] = drone.get_log('range.back')
        range_sensors[0] = drone.get_log('range.left')
        range_sensors[1] = drone.get_log('range.right')
        position_estimate[1] = drone.get_log('stateEstimate.x')
        position_estimate[0] = drone.get_log('stateEstimate.y')
        RIGHT = 1
        LEFT = -1
    elif direction == Direction.RIGHT:
        range_sensors[2] = drone.get_log('range.front')
        range_sensors[3] = drone.get_log('range.back')
        range_sensors[1] = drone.get_log('range.left')
        range_sensors[0] = drone.get_log('range.right')
        position_estimate[1] = drone.get_log('stateEstimate.x')
        position_estimate[0] = drone.get_log('stateEstimate.y')
        RIGHT = -1
        LEFT = 1
    else:
        range_sensors[1] = drone.get_log('range.front')
        range_sensors[0] = drone.get_log('range.back')
        range_sensors[3] = drone.get_log('range.left')
        range_sensors[2] = drone.get_log('range.right')
        position_estimate[0] = drone.get_log('stateEstimate.x')
        position_estimate[1] = drone.get_log('stateEstimate.y')
        RIGHT = 1
        LEFT = -1
    range_sensors[4] = drone.get_log('range.up')

    # ====== évitement latéral =======
    if range_sensors[2] < AVOID_DIST_LAT:  # obstacle détecté à gauche
        drone.obstacle_lateral = True
        speed_east_lat = -AVOID_SPEED_LAT
    elif range_sensors[3] < AVOID_DIST_LAT:  # obstacle détecté à droite
        drone.obstacle_lateral = True
        speed_east_lat = AVOID_SPEED_LAT
    else:
        drone.obstacle_lateral = False
        speed_east_lat = 0


    # ====== évitement frontal ========
    if range_sensors[0] < AVOID_DIST_FRONT:
        drone.obstacle_frontal = True
        if position_estimate[1] > line_position + 0.4:  # trop a gauche doit éviter par la droite
            avoid_dir = RIGHT
            drone.default_direction = RIGHT
        elif position_estimate[1] < line_position - 0.4:  # trop a droite doit éviter par la gauche
            avoid_dir = LEFT
            drone.default_direction = LEFT

        speed_east_front = avoid_dir * AVOID_SPEED_FRONT
    else:
        speed_east_front = 0
        drone.obstacle_frontal = False


    # ====== update la vitesse en y ======
    speed_east = (speed_east_lat + speed_east_front)

    if speed_east != 0:
        correction = FORWARD_SPEED/2  # correction pour aller plus lentement quand il y a des obstacles
    
    # ====== revient à la ligne directrice si pas d'obstacle ===============
    if drone.obstacle_wait == False:
        correction = 0
        if position_estimate[1] > line_position + POSITION_DIRECTION_THRESH:
            print(f'Pos estimate > : {position_estimate[1]:.3f}')
            speed_east =  RIGHT * AVOID_SPEED_COME_BACK
        elif position_estimate[1] < line_position - POSITION_DIRECTION_THRESH:
            speed_east =  LEFT * AVOID_SPEED_COME_BACK
            print(f'Pos estimate < : {position_estimate[1]:.3f}')
        else:
            speed_east = 0
        print(speed_east)

    # ===== Délai avant la déclaration de fin d'obstacle pour revenir sur la ligne de direction
    if drone.obstacle_frontal or drone.obstacle_lateral:
        drone.time1 = time.time_ns()
        drone.obstacle_wait = True
    if time.time_ns() - drone.time1 > AVOID_TIME_COME_BACK*1e9:
        drone.obstacle_wait = False
        
    speed_north = FORWARD_SPEED-correction

    if direction == Direction.FORWARD:
        speed_x, speed_y = speed_north, speed_east
    elif direction == Direction.LEFT:
        speed_x, speed_y = -speed_east, speed_north
    elif direction == Direction.RIGHT:
        speed_x, speed_y = speed_east, -speed_north
        print("\n")
        print(f'Speed east {speed_east}')
        print(speed_north)
    elif direction == Direction.BACKWARD:
        speed_x, speed_y = -speed_north, -speed_east
    
    return speed_x, speed_y

def start_zone_2_check(drone:Drone):
    """ vérifie quand le drone arrive dans la zone de la plateforme d'arrivée """

    arrival = False

    if drone.get_log('stateEstimate.x') >= arena.START_ZONE_2:
        arrival = True

    return arrival


def main_crossing(drone:Drone):
    fly = True

    #le drone suit la ligne au centre de l'arène
    central_line = arena.ORIGIN_Y - arena.WIDTH/2

    while(fly):        
        drone.stop_by_hand()
        arrival = start_zone_2_check(drone)
        if not arrival:
            speed_x, speed_y = avoid(drone, 0.3, Direction.LEFT)
            drone.start_linear_motion(speed_x, speed_y, 0)
            # drone.slam.slam_update() # quoi mettre en paramètre ?
        else:
            drone.stop()
            fly = False
        time.sleep(0.1)
    # drone.slam.slam_hold()



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
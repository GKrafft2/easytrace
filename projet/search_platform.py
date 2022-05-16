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

class avoidance:
    right = -1
    left = 1

    avoid_dir = right  # drone va commencer par éviter les obstacles par la droite

def search(drone:Easytrace):
    
    #search zone
    CORNER = [[0,0.5],[0,1]]

    position_estimate = [0, 0]
    position_estimate[0] = drone.get_log('stateEstimate.x')
    position_estimate[1] = drone.get_log('stateEstimate.y')
    state = 0
    OFFSET_X = 0.3
    fly = 1

    distance_y = 1
    distance_x = 0.2

    while(fly):
        position_estimate[0] = drone.get_log('stateEstimate.x')
        position_estimate[1] = drone.get_log('stateEstimate.y')
        # print(position_estimate[1])
        # va au bord gauche
        if state == 0:
            move(drone, distance_y, direction_y=1)
            state = 1

        # avance de 10cm
        if state == 1:
            if (position_estimate[0] + OFFSET_X) > CORNER[0][1]:
                fly = 0
            move(drone, distance_x, direction_x=1)
            state = 2

        # va au bord droite
        if state == 2:
            move(drone, distance_y, direction_y=-1)
            state = 3

        # avance de 10cm
        if state == 3:
            if (position_estimate[0] + OFFSET_X) > CORNER[0][1]:
                fly = 0
            move(drone, distance_x, direction_x=1)
            state = 0

def move(drone:Easytrace, distance, direction_x=0, direction_y=0, speed_x=0.2, speed_y=0.2):
    # direction_x: avant=1, arrière=-1
    # direction_y: droite=-1, gauche=1
    position_estimate = [0, 0]
    position_estimate[0] = drone.get_log('stateEstimate.x')
    position_estimate[1] = drone.get_log('stateEstimate.y')

    start_position = [0, 0]
    start_position[0] = drone.get_log('stateEstimate.x')
    start_position[1] = drone.get_log('stateEstimate.y')

    reach = 0
    drone.start_linear_motion(direction_x*speed_x, direction_y*speed_y, 0)
    while(not reach):
        position_estimate[0] = drone.get_log('stateEstimate.x')
        position_estimate[1] = drone.get_log('stateEstimate.y')
        if not direction_y and direction_x == 1:
            if abs(start_position[0] + distance)<position_estimate[0]:
                reach = 1
                drone.stop
            print(start_position[0], 'start position')
            print(start_position[0] + distance, 'start+dist')
            print(position_estimate[1], 'pos estimate')
        if not direction_y and direction_x == -1:
            if abs(start_position[0] + direction_x*distance)>position_estimate[0]:
                reach = 1
                drone.stop
        if not direction_x and direction_y == 1:
            if abs(start_position[1] + distance)<position_estimate[1]:
                reach = 1
                drone.stop
            print(start_position[1] + distance, 'start+dist')
            print(position_estimate[1], 'pos estimate')
        if not direction_x and direction_y == -1:
            if abs(start_position[1] + direction_y*distance)>position_estimate[1]:
                reach = 1
                drone.stop
            print(start_position[1] + direction_y*distance, 'start+dist')
            print(position_estimate[1], 'pos estimate')



# def search(drone:Easytrace):
#     #search zone
#     CORNER = [[0,0.5],[0,1]]

#     start_position = [0, 0]
#     start_position[0] = drone.get_log('stateEstimate.x')
#     start_position[1] = drone.get_log('stateEstimate.y')

#     position_estimate = [0, 0]
#     position_estimate[0] = drone.get_log('stateEstimate.x')
#     position_estimate[1] = drone.get_log('stateEstimate.y')
#     state = 0
#     speed_x = [0,0.1,0,0.1]
#     speed_y = [0.2,0,-0.2,0]
#     ERROR = 0.02

#     pos_y_goal = [CORNER[1][0]]
#     OFFSET_X = 0.1
#     fly = 1

#     while(fly):
#         start_position[0] = drone.get_log('stateEstimate.x')
#         start_position[1] = drone.get_log('stateEstimate.y')
#         drone.start_linear_motion(speed_x[state], speed_y[state], 0)
#         pos_reach = 0
#         while(not pos_reach):
#             position_estimate[0] = drone.get_log('stateEstimate.x')
#             position_estimate[1] = drone.get_log('stateEstimate.y')
#             # print(position_estimate[1])
#             print(position_estimate[0])
#             # va au bord gauche
#             if state == 0:
#                 if abs(position_estimate[1] - CORNER[1][1])<ERROR:
#                     pos_reach = 1
#                     state = 1

#             # avance de 10cm
#             if state == 1:
#                 if (start_position[0] + OFFSET_X) > CORNER[0][1]:
#                     pos_reach = 1
#                     fly = 0
#                 if abs(start_position[0] + OFFSET_X) < position_estimate[0]:
#                     pos_reach = 1
#                     state = 2

#             # va au bord droite
#             if state == 2:
#                 if abs(position_estimate[1] - CORNER[1][0])<ERROR:
#                     pos_reach = 1
#                     state = 3

#             # avance de 10cm
#             if state == 3:
#                 if (start_position[0] + OFFSET_X) > CORNER[0][1]:
#                     pos_reach = 1
#                     fly = 0
#                 if abs(start_position[0] + OFFSET_X)<position_estimate[0]:
#                     pos_reach = 1
#                     state = 0




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

        # fly_while_avoid(drone)
        search(drone)

        print("land")
        drone.land()

        drone.stop_logs(save=False)
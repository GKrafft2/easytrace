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

def edge_detection(drone:Drone):

    # Alias pour les logs de drone estimate.x estimate.y et estimate.z
    # Permet de garder les mêmes valeurs durant un passage complet de boucle
    # Un appel direct aux logs peut être actualisé entre temps
    position_estimate = [0, 0, 0]
    edge_detected = False
    zrange = np.zeros(5)

    position_estimate[0] = drone.get_log('stateEstimate.x') 
    position_estimate[1] = drone.get_log('stateEstimate.y')
    position_estimate[2] = drone.get_log('stateEstimate.z')

    # Tableau "circulaire" des 5 derniers logs de estimate.z qui sont plus élevé que default_height
    if position_estimate[2] > drone.height_cmd-0.02:
        zrange = np.append(zrange, position_estimate[2])
        zrange = zrange[1:]

    # Détecte un changement de hauteur selon le threshold = détection de la plateforme
    THRESH = 0.013
    moy = np.mean(zrange[:-1])
    if moy > drone.height_cmd-0.05 and (zrange[-1] < moy - THRESH or zrange[-1] > moy + THRESH): #detecte si on est passé au dessus de qqch (plateforme)
        #le mode landing est activé
        print("edge found")
        edge_detected = True
        drone.stop()

    # retourne la position position détectée
    return edge_detected, position_estimate[0], position_estimate[1]

def search(drone:Drone):
    
    #search zone
    CORNER = [[0,0.5],[0,1]]

    position_estimate = [0, 0]
    position_estimate[0] = drone.get_log('stateEstimate.x')
    position_estimate[1] = drone.get_log('stateEstimate.y')
    OFFSET_X = 0.3
    fly = 1

    distance_y = 1
    distance_x = 0.2
    drone.height_cmd = drone.get_log('stateEstimate.z')


    while(fly):
        position_estimate[0] = drone.get_log('stateEstimate.x')
        position_estimate[1] = drone.get_log('stateEstimate.y')
        # print(position_estimate[1])

        # va au bord gauche
        move(drone, distance_y, direction_y=1)

        # attéri si fin de limitation
        if (position_estimate[0] + OFFSET_X) > CORNER[0][1]:
            fly = 0
        # avance de 10cm
        move(drone, distance_x, direction_x=1)

        # va au bord droite
        move(drone, distance_y, direction_y=-1)

        # avance de 10cm
        move(drone, distance_x, direction_x=1)

def move(drone:Drone, distance, direction_x=0, direction_y=0, speed_x=0.2, speed_y=0.2):
    # direction_x: avant=1, arrière=-1
    # direction_y: droite=-1, gauche=1
    position_estimate = [0, 0]
    position_estimate[0] = drone.get_log('stateEstimate.x')
    position_estimate[1] = drone.get_log('stateEstimate.y')

    start_position = [0, 0]
    start_position[0] = drone.get_log('stateEstimate.x')
    start_position[1] = drone.get_log('stateEstimate.y')

    reach = 0
    edge_detected = 0
    drone.start_linear_motion(direction_x*speed_x, direction_y*speed_y, 0)
    while(not reach and not edge_detected):
        # edge_detected, _, _ = edge_detection(drone)
        position_estimate[0] = drone.get_log('stateEstimate.x')
        position_estimate[1] = drone.get_log('stateEstimate.y')
        if not direction_y and direction_x == 1:
            if abs(start_position[0] + distance)<position_estimate[0]:
                reach = 1
                drone.stop()
        if not direction_y and direction_x == -1:
            if abs(start_position[0] + direction_x*distance)>position_estimate[0]:
                reach = 1
                drone.stop()
        if not direction_x and direction_y == 1:
            if abs(start_position[1] + distance)<position_estimate[1]:
                reach = 1
                drone.stop()
        if not direction_x and direction_y == -1:
            if abs(start_position[1] + direction_y*distance)>position_estimate[1]:
                reach = 1
                drone.stop()



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
        drone = Drone(scf, default_height=0.4)

        drone.start_logs()

        drone.take_off()

        # fly_while_avoid(drone)
        search(drone)

        print("land")
        drone.land()

        drone.stop_logs(save=False)
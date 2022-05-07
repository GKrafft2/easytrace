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


def search_edge(drone:Easytrace, speed_x, speed_y):
    print("METHOD : search_edge")

    # Alias pour les logs de drone estimate.x estimate.y et estimate.z
    # Permet de garder les mêmes valeurs durant un passage complet de boucle
    # Un appel direct aux logs peut être actualisé entre temps
    position_estimate = [0, 0, 0]

    zrange = np.zeros(5)
    box_found = False
    drone.speed_x_cmd = speed_x
    drone.speed_y_cmd = speed_y

    # Assure d'être à 40cm de hauteur pour chercher la platform
    LANDING_HEIGHT = 0.4
    if drone.height_cmd != LANDING_HEIGHT:
        drone.go_to_up(LANDING_HEIGHT)

    while(not box_found):

        # Met à jour les vitesses de déplacement
        drone.start_linear_motion(drone.speed_x_cmd, drone.speed_y_cmd, 0)
        # time.sleep(1)
        position_estimate[0] = drone.get_log('stateEstimate.x') 
        position_estimate[1] = drone.get_log('stateEstimate.y')
        position_estimate[2] = drone.get_log('stateEstimate.z')

        # Tableau "circulaire" des 5 derniers logs de estimate.z qui sont plus élevé que default_height
        if position_estimate[2] > LANDING_HEIGHT :
            zrange = np.append(zrange, position_estimate[2])
            zrange = zrange[1:]
            # print(zrange)

        # Détecte un changement de hauteur selon le threshold = détection de la plateforme
        THRESH = 0.010
        moy = np.mean(zrange[:-1])
        if moy > 0.395 and (zrange[-1] < moy - THRESH or zrange[-1] > moy + THRESH): #detecte si on est passé au dessus de qqch (plateforme)
            #le mode landing est activé
            print("edge found")
            box_found = True
            drone.stop()
        
        # Délai très important ! (faut le temps que les commandes soient envoyées au drone)
        time.sleep(0.1)

    # retourne la position position détectée
    return position_estimate[0], position_estimate[1]


def land_on_platform(drone:Easytrace):
    print("METHOD : land_on_platform")

    max_vel = 0.2
    error = 1

    fly = True
    drone.speed_x_cmd = 0.2
    drone.speed_y_cmd = -0.2
        
    #screenshot de la pos au moment ou il detecte la boite
    position_estimate = [0,0]
    position_estimate[0] = drone.get_log('stateEstimate.x') 
    position_estimate[1] = drone.get_log('stateEstimate.y')

    # on estime le centre de la box en fonction de là où il detecte un edge
    center_x = platform.x_start + np.sign(drone.speed_x_cmd) * platform.HALF
    center_y = platform.y_start - np.sign(drone.speed_y_cmd) * platform.HALF

    print(f'pos drone x = {position_estimate[0]:.3f} y = {position_estimate[1]:.3}')
    print(f'pos boite x = {center_x:.3f} y = {center_y:.3f}')
    print(f'Platform start x = {platform.x_start:.3f} y = {platform.y_start:.3f}')

    while(fly):

        position_estimate[0] = drone.get_log('stateEstimate.x') 
        position_estimate[1] = drone.get_log('stateEstimate.y')
        # print(f'error {error}')
        # essaye de revenir au centre de la boite
        if error > 0.005 :
            landing_speed = -0.1
            P = 6
            err_x = (position_estimate[0]-center_x)
            err_y = (position_estimate[1]-center_y)
            landing_speed_x = err_x * landing_speed * P
            landing_speed_y = err_y * landing_speed * P

            # anti-windup
            # if landing_speed_x > 0.2:
            #     landing_speed_x = 0.2
            # if landing_speed_y > 0.2:
            #     landing_speed_y = 0.2
            # print(landing_speed_x, landing_speed_y)

            drone.start_linear_motion(landing_speed_x,landing_speed_y, 0)
            error = np.sqrt(err_x**2+err_y**2)

        # si l erreur est suffisamment petite on atterit
        else:
            print(f'pos drone x = {position_estimate[0]:.3f} y = {position_estimate[1]:.3}')
            print('Goal achieved ')
            
            fly = False
            drone.stop()

        time.sleep(0.1)

        # print(f'x = {position_estimate[0]:.2f} y = {position_estimate[1]:.2f}')

    time.sleep(0.1)



if __name__ == '__main__':

    # initalise les drivers low level, obligatoire
    cflib.crtp.init_drivers()
    # identifiant radio du drone
    URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E702')

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:

        # crée un drone (hérite de motion commander)
        drone = Easytrace(scf, default_height=0.4)
        
        drone.start_logs()

        drone.take_off()

        platform.x_start, _ = search_edge(drone, 0.2, 0)
        drone.move_distance(0.08, 0, 0, 0.2)
        time.sleep(2)
        _, platform.y_start = search_edge(drone, 0, -0.12)  

        land_on_platform(drone)


        # ====== lié à double_passage_cercle.csv
        # drone.forward(0.8, velocity=0.1)
        # drone.circle_right(0.2, angle_degrees=90)
        # drone.forward(0.3)
        # drone.circle_right(0.2, angle_degrees=90)
        # drone.forward(0.5)
        # drone.circle_right(0.2, angle_degrees=90)
        # drone.forward(1)


        print("land")
        drone.land()

        drone.stop_logs(save=False)
        
       
        



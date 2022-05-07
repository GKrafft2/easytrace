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


def search_platform(drone:Easytrace):
    print("METHOD : search_platform")

    # Alias pour les logs de drone estimate.x estimate.y et estimate.z
    # Permet de garder les mêmes valeurs durant un passage complet de boucle
    # Un appel direct aux logs peut être actualisé entre temps
    position_estimate = [0, 0, 0]

    zrange = np.zeros(5)
    box_found = False
    drone.speed_x_cmd = 0.2
    drone.speed_y_cmd = 0

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
        position_estimate[2] = drone.get_log('range.zrange')

        # Tableau "circulaire" des 5 derniers logs de estimate.z qui sont plus élevé que default_height
        if position_estimate[2] > LANDING_HEIGHT*1000 :
            zrange = np.append(zrange, position_estimate[2])
            zrange = zrange[1:]
            print(zrange)

        # Détecte un changement de hauteur selon le threshold = détection de la plateforme
        THRESH = 10
        moy = np.mean(zrange[:-1])
        if moy > 395 and (zrange[-1] < moy - THRESH or zrange[-1] > moy + THRESH): #detecte si on est passé au dessus de qqch (plateforme)
            #le mode landing est activé
            print("box found")
            box_found = True
            drone.stop()
        
        # Délai très important ! (faut le temps que les cmd soient envoyées au drone)
        time.sleep(0.1)
        # print(f'z = {position_estimate[2]:.2f}')

    print("Platform found, landing")
    drone.stop()
    time.sleep(1)

def detect_ascend(drone:Easytrace):
    detected = False
    return detected

def detect_descend(drone:Easytrace):
    pass

def land_on_platform(drone:Easytrace):
    print("METHOD : land_on_platform")

    max_vel = 0.2
    error = 1

    fly = True
    drone.speed_x_cmd = 0.2
    drone.speed_y_cmd = 0
        
    #screenshot de la pos au moment ou il detecte la boite
    position_estimate = [0,0]
    position_estimate[0] = drone.get_log('stateEstimate.x') 
    position_estimate[1] = drone.get_log('stateEstimate.y')

    # on estime le centre de la box en fonction de là où il detecte un edge
    center_x = position_estimate[0] + np.sign(drone.speed_x_cmd) * platform.HALF.value
    center_y = position_estimate[1] + np.sign(drone.speed_y_cmd) * platform.HALF.value

    print(f'pos drone x = {position_estimate[0]:.2f} y = {position_estimate[1]:.2}')
    print(f'pos boite x = {center_x:.2f} y = {center_y:.2f}')

    while(fly):

        position_estimate[0] = drone.get_log('stateEstimate.x') 
        position_estimate[1] = drone.get_log('stateEstimate.y')
        print(f'error {error}')
        # essaye de revenir au centre de la boite
        if error > 0.02 :
            landing_speed = -0.1
            P = 5
            err_x = (position_estimate[0]-center_x)
            err_y = (position_estimate[1]-center_y)
            landing_speed_x = err_x * landing_speed * P
            landing_speed_y = err_y * landing_speed * P

            # eviter que les vitesses deviennent trop grandes marche pas psk on risque d avoir des vitesses negatives
            #if landing_speed_x > 0.2:
                #landing_speed_x = 0.2
            #if landing_speed_y > 0.2:
                #landing_speed_y = 0.2
            print(landing_speed_x, landing_speed_y)

            drone.start_linear_motion(landing_speed_x,landing_speed_y, 0)
            error = np.sqrt(err_x**2+err_y**2)

        # si l erreur est suffisamment petite on atterit
        else:
            print('Goal achieved ')
            fly = False
            drone.stop()

        time.sleep(0.1)

        print(f'x = {position_estimate[0]:.2f} y = {position_estimate[1]:.2f}')

    time.sleep(0.1)


def move_linear_simple(drone:Easytrace):
    drone.go_to_forward(4)  

def take_off_land(drone:Easytrace):

    drone.take_off(0.3)
    print(drone.get_log('stateEstimate.z'))
    time.sleep(2)
    drone.land()
    print("landed")



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

        # move_linear_complex(drone)
        # move_linear_simple(drone)
        # drone.go_to_up(0.4)
        search_platform(drone)
        # time.sleep(5)
        land_on_platform(drone)      

        print("land")
        drone.land(1)

        drone.stop_logs(save=False)
        
       
        



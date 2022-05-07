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
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils import uri_helper
from matplotlib.pyplot import disconnect

# Libraries personnelles
from Easytrace_drone import Easytrace
from arena import arena_dim, arena_limits, platform


def search_platform(drone:Easytrace):

    # Alias pour les logs de drone estimate.x estimate.y et estimate.z
    # Permet de garder les mêmes valeurs durant un passage complet de boucle
    # Un appel direct aux logs peut être actualisé entre temps
    position_estimate = [0, 0, 0]

    zrange = np.zeros(5)
    box_not_found = True
    drone.speed_x_cmd = 0.2
    drone.speed_y_cmd = 0

    # Assure d'être à 40cm de hauteur pour chercher la platform
    LANDING_HEIGHT = 0.4
    if drone.height_cmd != LANDING_HEIGHT:
        drone.go_to_up(LANDING_HEIGHT)

    while(box_not_found):

        # Met à jour les vitesses de déplacement
        drone.start_linear_motion(drone.speed_x_cmd, drone.speed_y_cmd, 0)

        position_estimate[0] = drone.get_log('stateEstimate.x') 
        position_estimate[1] = drone.get_log('stateEstimate.y')
        position_estimate[2] = drone.get_log('stateEstimate.z')

        # Tableau "circulaire" des 5 derniers logs de estimate.z qui sont plus élevé que default_height
        if position_estimate[2] > LANDING_HEIGHT:
            zrange = np.append(zrange, position_estimate[2])
            zrange = zrange[1:]
            print(zrange)

        # Détecte un changement de hauteur selon le threshold = détection de la plateforme
        THRESH = 0.008
        moy = np.mean(zrange[:len(zrange) - 1])
        if moy > 0.395 and (zrange[len(zrange) - 1] < moy - THRESH or zrange[len(zrange) - 1] > moy + THRESH): #detecte si on est passé au dessus de qqch (plateforme)
            #le mode landing est activé
            box_not_found = False

        # print(f'z = {position_estimate[2]:.2f}')

    print("Platform found, landing")
    drone.stop()
    time.sleep(1)

def land_on_platform(drone:Easytrace):

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
    center_y = position_estimate[1] + np.sign(drone.speed_x_cmd) * platform.HALF.value

    print(f'pos boite = {center_x} y = {center_y}')

    while(fly):

        print(f'error {error}')
        # essaye de revenir au centre de la boite
        if error > 0.05 :
            landing_speed = 0.1
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
            print('boite')
            fly = False
            drone.land()

        # mc.start_linear_motion(-body_x_cmd,-body_y_cmd, 0)#revient sur ses pas
        # time.sleep(1)#laisse le temps de revenir sur la plateforme
        # break#quitte le while donc atterit

        print(f'x = {position_estimate[0]:.2f} y = {position_estimate[1]:.2f}')

    time.sleep(0.1)


def move_linear_simple(drone:Easytrace):
    ...

def take_off_simple(drone:Easytrace):

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
        # wait for the drone to stabilize
        time.sleep(1)

        search_platform(drone)
        land_on_platform(drone)
        print("land")
        time.sleep(1)
        drone.land()
        # take_off_simple(drone)

        drone.stop_logs(save=False)
        
       
        



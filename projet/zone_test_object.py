import logging
import sys
import os
import time
import datetime as dt
from threading import Event

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils import uri_helper
from matplotlib.pyplot import disconnect
from Easytrace_drone import Easytrace

import numpy as np

URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E702')

DEFAULT_HEIGHT = 0.4
BOX_LIMIT = 0.3


position_estimate = [0, 0]

zrange = np.zeros(5)
print(zrange)
landing = False


def move_box_limit(scf, drone):
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:

        time.sleep(1)

        body_x_cmd = 0.2
        body_y_cmd = 0
        max_vel = 0.2
        error = 1

        demi_boite = 0.10

        fly = True
        first_pass = True

        while (fly):

            #print(data)
            global position_estimate
            global zrange
            global landing

            position_estimate[0] = drone.logs[drone.count-1][0]
            position_estimate[1] = drone.logs[drone.count-1][1]

            if drone.logs[drone.count][2] > DEFAULT_HEIGHT:
                zrange = np.append(zrange, drone.logs[drone.count][2])
                zrange = zrange[1:]
                #print(zrange)

            THRESH = 0.013
            moy = np.mean(zrange[:len(zrange) - 1])
            if moy > 0.395 and (zrange[len(zrange) - 1] < moy - THRESH or zrange[len(zrange) - 1] > moy + THRESH): #detecte si on est passé au dessus de qqch (plateforme)
                #print('boite')
                landing = True#le mode landing est activé -> sera utlisé dans move_box_limit


            print(f'x = {position_estimate[0]} y = {position_estimate[1]}')
            print('dans while(fly)')
            if landing:
                print('landing')
                #screenshot de la pos au moment ou il detecte la boite
                if first_pass:
                    limit_x = position_estimate[0]
                    limit_y = position_estimate[1]

                    # on estime le centre de la box en fonction de la ou il detecte un edge et sa vitesse d'approche
                    center_x = limit_x + np.sign(body_x_cmd) * demi_boite
                    center_y = limit_y + np.sign(body_y_cmd) * demi_boite

                    print(f'pos boite = {center_x} y = {center_y}')
                    first_pass = False

                time.sleep(0.5)

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

                    mc.start_linear_motion(landing_speed_x,landing_speed_y, 0)
                    error = np.sqrt(err_x**2+err_y**2)
                # si l erreur est suffisamment petite on atterit en breakant
                else:
                    print('boite')
                    fly = False

                # mc.start_linear_motion(-body_x_cmd,-body_y_cmd, 0)#revient sur ses pas
                # time.sleep(1)#laisse le temps de revenir sur la plateforme
                # break#quitte le while donc atterit

            if not landing :
                mc.start_linear_motion(body_x_cmd, body_y_cmd, 0)

            time.sleep(0.1)


def move_linear_simple(scf):
    ...

def take_off_simple(scf):
    mc = MotionCommander(scf, default_height=DEFAULT_HEIGHT)
    mc.take_off(0.3)
    time.sleep(5)
    mc.land()
    print("landed")

def log_pos_callback(timestamp, data, logconf):
    #print(data)
    global position_estimate
    global zrange
    global landing

    position_estimate[0] = data['stateEstimate.x']
    position_estimate[1] = data['stateEstimate.y']
    if data['stateEstimate.z'] > DEFAULT_HEIGHT:
        zrange = np.append(zrange,data['stateEstimate.z'])
        zrange = zrange[1:]
        #print(zrange)

    THRESH = 0.013
    moy = np.mean(zrange[:len(zrange) - 1])
    if moy > 0.395 and (zrange[len(zrange) - 1] < moy - THRESH or zrange[len(zrange) - 1] > moy + THRESH): #detecte si on est passé au dessus de qqch (plateforme)
        #print('boite')
        landing = True#le mode landing est activé -> sera utlisé dans move_box_limit




if __name__ == '__main__':

    # initalise les drivers low level, obligatoire
    cflib.crtp.init_drivers()

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:

        drone = Easytrace(scf.cf)

        # drone.logconf.data_received_cb.add_callback(log_pos_callback)

        drone.logconf.start()
        move_box_limit(scf, drone)
        # take_off_simple(scf)
        drone.logconf.stop()


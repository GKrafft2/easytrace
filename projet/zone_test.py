import logging
import sys
import time
from threading import Event

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.positioning.position_hl_commander import PositionHlCommander
from cflib.utils import uri_helper

import numpy as np

URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E702')

DEFAULT_HEIGHT = 0.3
BOX_LIMIT = 0.3

deck_attached_event = Event()

logging.basicConfig(level=logging.ERROR)

position_estimate = [0, 0]

zrange = np.zeros(5)
print(zrange)
landing = 0


def move_box_limit(scf):
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        body_x_cmd = 0.2
        body_y_cmd = 0.2
        max_vel = 0.2
        middle_box = 0.2
        i=0

        while (1):
            #if position_estimate[0] > BOX_LIMIT:
            #    mc.start_back()
            #elif position_estimate[0] < -BOX_LIMIT:
            #    mc.start_forward()

            # if position_estimate[0] > BOX_LIMIT:
            #     body_x_cmd = -max_vel
            # if position_estimate[0] < -BOX_LIMIT:
            #     body_x_cmd = max_vel
            # if position_estimate[1] > BOX_LIMIT:
            #     body_y_cmd = -max_vel
            # if position_estimate[1] < -BOX_LIMIT:
            #     body_y_cmd = max_vel

            if landing == 1:
                print('landing')
                #time.sleep(0.5)
                #pc.go_to(position_estimate[0]+middle_box,position_estimate[1],DEFAULT_HEIGHT)
                #mc.start_linear_motion(-body_x_cmd,-body_y_cmd, 0)#revient sur ses pas
                if i == 0:
                    mc.forward(middle_box)
                    time.sleep(1)#laisse le temps de revenir sur la plateforme
                mc.start_linear_motion(0,body_y_cmd, 0)
                time.sleep(2)
            else:
                mc.start_linear_motion(body_x_cmd, 0, 0)

                
            if landing >= 2:
                print('landing')
                #time.sleep(0.5)
                #pc.go_to(position_estimate[0],position_estimate[1]+middle_box,DEFAULT_HEIGHT)
                #mc.start_linear_motion(-body_x_cmd,-body_y_cmd, 0)#revient sur ses pas
                mc.left(middle_box)
                time.sleep(1)#laisse le temps de revenir sur la plateforme
                break#quitte le while donc atterit


            time.sleep(0.1)


def move_linear_simple(scf):
    ...

def take_off_simple(scf):
    ...

def log_pos_callback(timestamp, data, logconf):
    #print(data)
    global position_estimate
    global zrange
    global landing

    if data['range.zrange'] > DEFAULT_HEIGHT*1000:
        zrange = np.append(zrange,data['range.zrange'])
        zrange = zrange[1:]
        #print(zrange)

    THRESH = 8
    moy = np.mean(zrange[:len(zrange) - 1])
    if moy > 295 and (zrange[len(zrange) - 1] < moy - THRESH or zrange[len(zrange) - 1] > moy + THRESH): #detecte si on est passé au dessus de qqch (plateforme)
        position_estimate[0] = data['stateEstimate.x']
        position_estimate[1] = data['stateEstimate.y']
        print('boite')
        landing += 1 #le mode landing est activé -> sera utlisé dans move_box_limit





def param_deck_flow(_, value_str):
    value = int(value_str)
    #print(value)
    if value:
        deck_attached_event.set()
        print('Deck is attached!')
    else:
        print('Deck is NOT attached!')


if __name__ == '__main__':
    cflib.crtp.init_drivers()

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:

        scf.cf.param.add_update_callback(group='deck', name='bcFlow2',
                                         cb=param_deck_flow)
        time.sleep(1)

        logconf = LogConfig(name='Position', period_in_ms=10)
        logconf.add_variable('stateEstimate.x', 'float')
        logconf.add_variable('stateEstimate.y', 'float')
        logconf.add_variable('range.zrange', 'uint16_t')
        scf.cf.log.add_config(logconf)
        logconf.data_received_cb.add_callback(log_pos_callback)


        logconf.start()
        move_box_limit(scf)
        logconf.stop()
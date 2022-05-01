"""
This script shows the basic use of the PositionHlCommander class.

Simple example that connects to the crazyflie at `URI` and runs a
sequence. This script requires some kind of location system.

The PositionHlCommander uses position setpoints.

Change the URI variable to your Crazyflie configuration.
"""
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.position_hl_commander import PositionHlCommander
from cflib.utils.multiranger import Multiranger
from cflib.utils import uri_helper
from cflib.positioning.motion_commander import MotionCommander

import sys
import time

from threading import Event #check ce que c est
import logging
import numpy as np

# URI to the Crazyflie to connect to
uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E702')
URI = uri


def slightly_more_complex_usage():
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        with PositionHlCommander(
                scf,
                x=0.0, y=0.0, z=0.0,
                default_velocity=0.3,
                default_height=0.5,
                controller=PositionHlCommander.CONTROLLER_MELLINGER) as pc:
            # Go to a coordinate
            pc.go_to(1.0, 1.0, 1.0)

            # Move relative to the current position
            pc.right(1.0)

            # Go to a coordinate and use default height
            pc.go_to(0.0, 0.0)

            # Go slowly to a coordinate
            pc.go_to(1.0, 1.0, velocity=0.2)

            # Set new default velocity and height
            pc.set_default_velocity(0.3)
            pc.set_default_height(1.0)
            pc.go_to(0.0, 0.0)


def simple_sequence():
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        with PositionHlCommander(scf, controller=PositionHlCommander.CONTROLLER_MELLINGER) as pc:
            with Multiranger(scf) as multiranger:
                for i in range(4):
                    pc.forward(1.0)
                    pc.left(1.0)
                    pc.back(1.0)
                    pc.right(1.0)

DEFAULT_HEIGHT = 0.3

deck_attached_event = Event()
logging.basicConfig(level=logging.ERROR)









def move_linear_simple(scf):
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        time.sleep(1)
        mc.forward(1,velocity=0.5)
        time.sleep(1)
        mc.turn_left(180)
        time.sleep(1)
        mc.forward(1,velocity=0.5)
        time.sleep(1)
        mc.turn_left(180)
        time.sleep(1)


def take_off_simple(scf):
    ...

def param_deck_flow(name, value_str):
   ...


if __name__ == '__main__':
    cflib.crtp.init_drivers()

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:

        scf.cf.param.add_update_callback(group='deck', name='bcFlow2',
                                         cb=param_deck_flow)
        time.sleep(1)

        move_linear_simple(scf)



#if __name__ == '__main__':
    #cflib.crtp.init_drivers()

    #simple_sequence()
    # slightly_more_complex_usage()

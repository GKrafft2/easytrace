from email.policy import default
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

import numpy as np

URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E702')

# ADMIN: Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)


class Easytrace(MotionCommander):
    def __init__(self, scf, default_height):
        # Initialise le MotionCommander
        super().__init__(scf, default_height=default_height)

        # Crazyflie instance
        self._cf = scf.cf

        # Variable for the deck flow connection
        self._deck_attached_event = Event()

        # ADMIN: Connect some callbacks from the Crazyflie API
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.param.add_update_callback(group='deck', name='bcFlow2', cb=self._param_deck_flow)
        time.sleep(1)

        # Variables à enregistrer
        self.logconf = LogConfig(name='Stabilizer', period_in_ms=10)
        # self.logconf.add_variable('range.front', 'uint16_t')
        # self.logconf.add_variable('range.back', 'uint16_t')
        # self.logconf.add_variable('range.up', 'uint16_t')
        # self.logconf.add_variable('range.left', 'uint16_t')
        # self.logconf.add_variable('range.right', 'uint16_t')
        self.logconf.add_variable('stateEstimate.x', 'float')  # estimated X coordinate
        self.logconf.add_variable('stateEstimate.y', 'float')  # estimated Y coordinate
        self.logconf.add_variable('stateEstimate.z', 'float')  # estimated Z coordinate

        # Matrice des logs, s'adapte en fonction du nombre de variables ajoutées
        self.logs_size = 100000
        self.logs = np.zeros([self.logs_size, len(self.logconf.variables)])
        # Indique la position du dernier log
        self.count = 0 

        # Adding the configuration cannot be done until a Crazyflie is
        # connected, since we need to check that the variables we
        # would like to log are in the TOC.
        try:
            self._cf.log.add_config(self.logconf)
            # This callback will receive the data
            self.logconf.data_received_cb.add_callback(self._stab_log_data)
            # This callback will be called on errors
            self.logconf.error_cb.add_callback(self._stab_log_error)
            # Start the logging
            # self.logconf.start()
            time.sleep(1)
        except KeyError as e:
            print('Could not start log configuration, {} not found in TOC'.format(str(e)))
        except AttributeError:
            print('Could not add log config, bad configuration.')

    def _disconnected(self, link_uri):
        """Callback when the Crazyflie is disconnected (called in all cases)"""
        print('Disconnected from %s' % link_uri)

        # sauvegarde les logs s'il y en a
        if self.count != 0:
            self.save_logs()

    def save_logs(self):
        # Get timestamp
        filename = dt.datetime.now().strftime("%Y_%m_%d_%H_%M_%S.csv")
        # Save log to file
        if not os.path.exists('logs'):
            os.makedirs('logs')
        filepath = os.path.join(os.getcwd(), 'logs', filename)
        np.savetxt(filepath, self.logs, delimiter=',')

    def _stab_log_data(self, timestamp, data, logconf):
        """ Callback from the log API when data arrives """
        # print('[%d][%s]: %s' % (timestamp, logconf.name, data))
        
        # Save info into log variable
        for idx, i in enumerate(list(data)):
            self.logs[self.count][idx] = data[i]

        self.count += 1

        # Soulève une erreur si on atteinds la fin de la matrice
        if self.count == self.logs_size:
            raise Exception('Log matrix is full')

    def _stab_log_error(self, logconf, msg):
        """ Callback from the log API when an error occurs """
        print('Error when logging %s: %s' % (logconf.name, msg))

    def _param_deck_flow(self, _, value_str):
        value = int(value_str)
        print(value)
        if value:
            self._deck_attached_event.set()
            print('Deck is attached!')
        else:
            print('Deck is NOT attached!')

    def get_log(self, variable):
        """ Return the latest log of the given variable index"""
        return self.logs[self.count-1][variable]

    def start_logs(self):
        """ Start sending logs """
        self.logconf.start()
    
    def stop_logs(self):
        """ Stop sending logs """
        self.logconf.stop()
        self.save_logs()
        self.logs *= 0
        self.count = 0
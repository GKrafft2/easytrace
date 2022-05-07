from email.policy import default
import logging
import os
import time
import datetime as dt
from threading import Event
from typing import Tuple

from cflib.crazyflie.log import LogConfig
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

        # ADMIN: Variable for the deck flow connection
        self._deck_attached_event = Event()

        # ADMIN: Connect some callbacks from the Crazyflie API
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.param.add_update_callback(group='deck', name='bcFlow2', cb=self._param_deck_flow)
        time.sleep(1)

        # Variables de commandes ()
        self.height_cmd = default_height
        self.speed_x_cmd = 0
        self.speed_y_cmd = 0

        # Variables d'état
        self.x = 0
        self.y = 0
        self.z = 0


        # Variables et types correspondants à logger
        # stateEstimate 'float' [m] (x, y, z, ...)
        # range 'uint16_t' [mm] (up, front, left, ...)
        self.logs_variables = ['stateEstimate.x', 'stateEstimate.y', 'stateEstimate.z']
        self._logs_variables_type = ['float', 'float', 'float']

        # Variables à enregistrer
        self.logconf = LogConfig(name='Stabilizer', period_in_ms=10)
        for variable, type in zip(self.logs_variables, self._logs_variables_type):
            self.logconf.add_variable(variable, type)

        # Matrice des logs, s'adapte en fonction du nombre de variables ajoutées
        self.logs_size = 100000
        self.logs = np.zeros([self.logs_size, len(self.logconf.variables)])
        # Indique la position du dernier log
        self.count = 0 

        try:
            self._cf.log.add_config(self.logconf)
            # ADMIN: This callback will receive the data
            self.logconf.data_received_cb.add_callback(self._stab_log_data)
            # ADMIN: This callback will be called on errors
            self.logconf.error_cb.add_callback(self._stab_log_error)
            time.sleep(1)
        except KeyError as e:
            print('Could not start log configuration, {} not found in TOC'.format(str(e)))
        except AttributeError:
            print('Could not add log config, bad configuration.')


    # ###############################################
    # ========= CRAZYFLY CALLBACK FUNCTIONS =========

    def _disconnected(self, link_uri):
        """Callback when the Crazyflie is disconnected (called in all cases)"""
        print('Disconnected from %s' % link_uri)

        # sauvegarde les logs s'il y en a
        if self.count != 0:
            self.save_logs()

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
        """ Check d'initialisation si le deck est bien attaché """
        value = int(value_str)
        print(value)
        if value:
            self._deck_attached_event.set()
            print('Deck is attached!')
        else:
            print('Deck is NOT attached!')


    # ###############################################
    # =============== LOG FUNCTIONS =================

    def get_log(self, variable, index=None):
        """ 
        Si index est donné, retourne le log correspondant
        Si array est vrai, retourne une liste des derniers indexs
        Autrement retourne le dernier logs enregistré de l'index de variable donné """

        # retourne l'index de la variable de log
        idx_variable = self.logs_variables.index(variable)

        if index is None:
            idx_log = self.count-1
        else:
            idx_log = self.count+index
            
        return self.logs[idx_log][idx_variable]        

    def start_logs(self):
        """ Start l'enregistrement des logs """
        self.logconf.start()
    
    def stop_logs(self, save=True):
        """ Stop les logs. Enregistre automatiquement en CSV et reset la matrice des logs """
        self.logconf.stop()
        if save:
            self.save_logs()
            self.logs *= 0
            self.count = 0

    def save_logs(self):
        """ Enregistre la matrice des logs en fichiers CSV"""
        # Get timestamp
        filename = dt.datetime.now().strftime("%Y_%m_%d_%H_%M_%S.csv")
        # Save log to file
        if not os.path.exists('logs'):
            os.makedirs('logs')
        filepath = os.path.join(os.getcwd(), 'logs', filename)
        np.savetxt(filepath, self.logs, delimiter=',')


    # ###############################################
    # ============ MOVEMENTS FUNCTIONS ==============

    def go_to_up(self, distance):
        """ Déplacement en Z à une hauteur absolue
            TODO:ajouter un PID si précision nécessaire
        """
        self.up(distance - self.get_log('stateEstimate.z'))

    def go_to_forward(self, distance):
        """ Déplacement en Z à une hauteur absolue
            TODO:ajouter un PID si précision nécessaire
        """
        self.forward(distance - self.get_log('stateEstimate.x'))

    def go_to_back(self, distance):
        """ Déplacement en Z à une hauteur absolue
            TODO:ajouter un PID si précision nécessaire
        """
        self.back(distance - self.get_log('stateEstimate.x'))

    def go_to_left(self, distance):
        """ Déplacement en Z à une hauteur absolue
            TODO:ajouter un PID si précision nécessaire
        """
        self.left(distance - self.get_log('stateEstimate.y'))

    def go_to_right(self, distance):
        """ Déplacement en Z à une hauteur absolue
            TODO:ajouter un PID si précision nécessaire
        """
        self.right(distance - self.get_log('stateEstimate.y'))

    # surcharge de Motion Commander
    def take_off(self):
        # execute Motion Commander method
        super(Easytrace, self).take_off()
        # wait for the drone to stabilize
        time.sleep(1.5)

    # surcharge de Motion Commander
    def land(self, velocity=0.2):
        """ Vitesse de 1 = chute libre """
        # wait for the drone to stabilize
        time.sleep(1)
        # execute Motion Commander method
        super(Easytrace, self).land(velocity)


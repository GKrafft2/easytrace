""" Drone classe which inherite of MotionCommander """

import logging
import os
import time
import datetime as dt
from threading import Event

from cflib.crazyflie.log import LogConfig
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils import uri_helper
from arena import Arena

import numpy as np
import sys
from arena import Direction
from slam import Slam

URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E702')

# ADMIN: Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)



class Drone(MotionCommander):
    def __init__(self, scf, default_height):

        # Initialisation of MotionCommander
        super().__init__(scf, default_height=default_height)

        # Crazyflie instance
        self._cf = scf.cf
        
        # Slam
        self.slam = Slam()

        # ADMIN: Variable for the deck flow connection
        self._deck_attached_event = Event()

        # ADMIN: Connect some callbacks from the Crazyflie API
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.param.add_update_callback(group='deck', name='bcFlow2', cb=self._param_deck_flow)
        time.sleep(1)

        # commande variables ()
        self.height_cmd = default_height
        self.position_x_offset = 0
        self.position_y_offset = 0
        self.default_speed = 0.3
        self.speed_x = 0
        self.speed_y = 0
        self.speed_z = 0

        # State variables
        self.z_cmd = 0
        self.direction = Direction.FORWARD
        self.TRESHOLD_UP = 0.03
        self.THRESHOLD_DOWN = 0.03

        # Sensor variables
        self.range_sensors = np.empty(5)
        self.position_estimate = np.empty(2)

        # Position history of the position (for edge detection)
        self.zrange = np.zeros(5)
    
        # if the drone is on the tracking line
        self.on_track = True           


        # Variables and corespondant type to log
        # stateEstimate 'float' [m] (x, y, z, ...)
        # range 'uint16_t' [mm] (up, front, left, ...)
        self.logs_variables = ['stateEstimate.x', 'stateEstimate.y', 'stateEstimate.z',
                               'range.front', 'range.back', 'range.left', 'range.right', 'range.up','range.zrange']
        self._logs_variables_type = ['float', 'float', 'float',
                                     'uint16_t', 'uint16_t', 'uint16_t', 'uint16_t', 'uint16_t','uint16_t']

        # Variables to save
        self.logconf = LogConfig(name='Stabilizer', period_in_ms=10)
        for variable, type in zip(self.logs_variables, self._logs_variables_type):
            self.logconf.add_variable(variable, type)

        # Logs matrix, adapt in fonction of the variables number added
        self.logs_size = 100000
        self.logs = np.zeros([self.logs_size, len(self.logconf.variables)])
        # Show the position of the last log
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

        # save the logs if there is one
        if self.count != 0:
            self.save_logs()

    def _stab_log_data(self, timestamp, data, logconf):
        """ Callback from the log API when data arrives """

        # Save info into log variable
        for idx, i in enumerate(list(data)):
            self.logs[self.count][idx] = data[i]

        self.count += 1

        # Show an error if the end of the matrix is reached
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

    def get_log(self, variable, index=None, array=False):
        """ 
        If index is given, returns the corresponding log
        If array is true, returns a list of the last indexes
        Otherwise returns the last recorded log of the given variable index """

        # return the index of the log variable
        idx_variable = self.logs_variables.index(variable)

        if index is None :
            data = self.logs[self.count-1,idx_variable]
        else: # index is not None and array == False:
            data = self.logs[self.count-index,idx_variable]
        
        # offset to keep the position after landing
        if variable == 'stateEstimate.x':
            data += self.position_x_offset
        if variable == 'stateEstimate.y':
            data += self.position_y_offset

        return data

    def start_logs(self):
        """ Start the logs saving """
        self.logconf.start()
    
    def stop_logs(self, save=True):
        """ Stop the logs. Save automatically in CSV and restet the logs matrix """
        self.logconf.stop()
        if save:
            self.save_logs()
            self.logs *= 0
            self.count = 0

    def save_logs(self):
        """ Save the logs matrix in CSV files """
        # Get timestamp
        filename = dt.datetime.now().strftime("%Y_%m_%d_%H_%M_%S.csv")
        # Save log to file
        if not os.path.exists('logs'):
            os.makedirs('logs')
        filepath = os.path.join(os.getcwd(), 'logs', filename)
        np.savetxt(filepath, self.logs, delimiter=',')


    # ###############################################
    # ============= STATES FUNCTIONS ================
    def __save_position_landing(self):
        self.position_x_offset = self.get_log('stateEstimate.x')
        self.position_y_offset = self.get_log('stateEstimate.y')

    def refresh_logs(self):
        self.range_sensors[0] = self.get_log('range.front')
        self.range_sensors[1] = self.get_log('range.back')
        self.range_sensors[2] = self.get_log('range.left')
        self.range_sensors[3] = self.get_log('range.right')
        self.range_sensors[4] = self.get_log('range.up')

        self.position_estimate[0] = self.get_log('stateEstimate.x')
        self.position_estimate[1] = self.get_log('stateEstimate.y')

    def update_slam(self):
        self.refresh_logs()
        self.slam.update(self.range_sensors, self.position_estimate)

    def get_time(self):
        return self.__time

    def update_time(self):
        self.__time = time.time_ns()/1e9

    # ###############################################
    # ============ MOVEMENTS FUNCTIONS ==============
    def go_to(self, x=None, y=None, z=None, velocity=0.2):
        """ Movement to an absolute position
        """
        
        if x is None:
            x = 0
        else:
            x, _, _ = self.__go_to_limits_check(x=x)
            x = x - self.get_log('stateEstimate.x')
        if y is None:
            y = 0
        else:
            _, y, _ = self.__go_to_limits_check(y=y)
            y = y - self.get_log('stateEstimate.y')
        if z is None:
            z = 0
        else:
            _,_, z = self.__go_to_limits_check(z=z)
            z = z - self.get_log('stateEstimate.z')

        self.move_distance(x, y, z, velocity)

    def __go_to_limits_check(self, x=0, y=0, z=0):
        """  check if the disired position is allowed and modified it if not """
        xp, yp, zp = x, y, z

        if x > Arena.LIM_NORTH:
            x = Arena.LIM_NORTH
        elif x < Arena.LIM_SOUTH:
            x = Arena.LIM_SOUTH
        if y > Arena.LIM_WEST:
            y = Arena.LIM_WEST
        elif y < Arena.LIM_EAST:
            y = Arena.LIM_EAST
        if z > Arena.LIM_UP:
            z = Arena.LIM_UP
        elif z < Arena.LIM_DOWN:
            z = Arena.LIM_DOWN

        if xp!=x or yp!=y or zp!=z:
            print("Go to command modified because outside limits, please check")

        return x, y, z

    def stop_by_hand(self):
        if self.get_log('range.up') < 200:
            self.stop()
            self.land()
            sys.exit()

    def stop_brutal(self):
        print(f' sx {self.speed_x}, sy {self.speed_y}')
        self.start_linear_motion(-self.speed_x, -self.speed_y, -self.speed_z)
        time.sleep(0.4)
        self.stop()
        time.sleep(0.5)

    # redefinition of Motion Commander
    def take_off(self, height=None):
        # execute Motion Commander method
        super(Drone, self).take_off(height=height)
        # start logging
        self.start_logs()
        # wait for the drone to stabilize
        time.sleep(2)

    # redefinition of Motion Commander
    def land(self, velocity=0.2):
        """ Vitesse de 1 = chute libre """
        # wait for the drone to stabilize
        time.sleep(1)
        # save coordinates
        self.__save_position_landing()
        # execute Motion Commander method
        super(Drone, self).land(velocity)

    def start_linear_motion(self, velocity_x_m, velocity_y_m, velocity_z_m, rate_yaw=0.0):

        self.speed_x = velocity_x_m
        self.speed_y = velocity_y_m
        self.speed_z = velocity_z_m
        # excecute Motion Commander method
        super(Drone, self).start_linear_motion(velocity_x_m, velocity_y_m, velocity_z_m, rate_yaw)

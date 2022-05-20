# Libraries python
import numpy as np
import cv2

# Libraries crazyflie
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper

from drone import Drone

from arena import Arena

class Slam():
    def __init__(self):
        self.map_shape = (500, 300)
        self.map = np.zeros(self.map_shape)
        self.offset_x = Arena.ORIGIN_X * 100
        self.offset_y = Arena.ORIGIN_Y * 100
        self.range_sensors = np.empty(5)
        self.position_estimate = [self.offset_x, self.offset_y]
        self.thresh_obstcl = 50
        self.blue = 128
        self.red = 255

    def update(self, range_sensor, position_estimates):
        self.range_sensors[0:4] = np.floor(range_sensor[0:4]/10)
        self.range_sensors[4] = range_sensor[4]
        self.position_estimate[0] = self.offset_x + np.floor(position_estimates[0] * 100)
        self.position_estimate[1] = self.offset_y + np.floor(position_estimates[1] * 100)

        # position mapping
        if self.position_estimate[0] < self.map.shape[0] and self.position_estimate[1] < self.map.shape[1]:
            self.map[int(self.position_estimate[0])][int(self.position_estimate[1])] = self.blue

        # obstcale mapping
        if self.position_estimate[0] + self.range_sensors[0] < self.map.shape[0]:
            if self.range_sensors[0] < self.thresh_obstcl:
                self.map[int(self.position_estimate[0] + self.range_sensors[0])][int(self.position_estimate[1])] = self.red  # front
        if self.position_estimate[0] - self.range_sensors[1] > 0:
            if self.range_sensors[1] < self.thresh_obstcl:
                self.map[int(self.position_estimate[0] - self.range_sensors[1])][int(self.position_estimate[1])] = self.red  # back
        if self.position_estimate[1] + self.range_sensors[2] < self.map.shape[1]:
            if self.range_sensors[2] < self.thresh_obstcl:
                self.map[int(self.position_estimate[0])][int(self.position_estimate[1] + self.range_sensors[2])] = self.red  # left
        if self.position_estimate[1] - self.range_sensors[3] > 0:
            if self.range_sensors[3] < self.thresh_obstcl:
                self.map[int(self.position_estimate[0])][int(self.position_estimate[1] - self.range_sensors[3])] = self.red  # right

        # update the map
        self.map_update()

    def hold(self):
        cv2.waitKey(0)

    def map_update(self):
        graymap = self.map.astype('uint8')
        colormap = cv2.applyColorMap(graymap, cv2.COLORMAP_JET)
        imS = cv2.resize(colormap, (self.map.shape[1], self.map.shape[0]))  # reshape
        imS = cv2.rotate(imS, cv2.ROTATE_180)  # rotate
        cv2.imshow('image', imS)
        cv2.waitKey(1)

# def fly_while_slam_class(drone:Drone, slam:Slam):
#     range_sensors = np.empty(5)
#     position_estimate = np.empty(2)
#     for i in range(3000):
#
#         range_sensors[0] = drone.get_log('range.front')
#         range_sensors[1] = drone.get_log('range.back')
#         range_sensors[2] = drone.get_log('range.left')
#         range_sensors[3] = drone.get_log('range.right')
#         range_sensors[4] = drone.get_log('range.up')
#
#         position_estimate[0] = drone.get_log('stateEstimate.x')
#         position_estimate[1] = drone.get_log('stateEstimate.y')
#
#         slam.update(range_sensors, position_estimate)
#
#         # L shaped path for testing
#         if i <= 1000 :
#             drone.start_linear_motion(0.2, 0, 0)
#         elif i > 1000 and i <= 1500 :
#             drone.start_linear_motion(0, 0.2, 0)
#         elif i > 1500 and i <= 2000 :
#             drone.start_linear_motion(0, -0.2, 0)
#         else :
#             drone.start_linear_motion(-0.2, 0, 0)
#
#     drone.land()
#     # le drone se pose et l affichage du SLAM reste actif
#     slam.hold()



if __name__ == '__main__':
    # initalise les drivers low level, obligatoire
    cflib.crtp.init_drivers()
    # identifiant radio du drone
    URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E702')

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        # crée un drone (hérite de motion commander)
        drone = Drone(scf, default_height=0.2)
        slam = Slam()

        drone.start_logs()

        drone.take_off()

        #fly_while_slam(drone)                  #without the slam class
        fly_while_slam_class(drone, slam)     # with the slam class

        print("land")
        drone.land()

        # drone.stop_logs(save=False)
# Libraries python
import numpy as np
import cv2

# Libraries crazyflie
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper

from arena import Arena

class Slam():
    def __init__(self):
        padding = 200
        self.map_shape = ((500+padding), (300+padding))
        self.map = np.zeros(self.map_shape)
        self.offset_x = Arena.ORIGIN_X * 100 + padding/2
        self.offset_y = Arena.ORIGIN_Y * 100 + padding/2
        self.range_sensors = np.empty(5)
        self.position_estimate = [self.offset_x, self.offset_y]
        self.thresh_obstcl = 50
        self.blue = 128
        self.red = 190
    
    def draw9(self,x,y,color):
        list = [-1,0,1]
        for a in list:
            for b in list:
                self.map[x+a][y+b] = color

    def update(self, range_sensor, position_estimates):
        self.range_sensors[0:4] = np.floor(range_sensor[0:4]/10)
        self.range_sensors[4] = range_sensor[4]
        self.position_estimate[0] = self.offset_x + np.floor(position_estimates[0] * 100)
        self.position_estimate[1] = self.offset_y + np.floor(position_estimates[1] * 100)

        # position mapping
        if self.position_estimate[0] < self.map.shape[0] and self.position_estimate[1] < self.map.shape[1]:
            self.draw9(int(self.position_estimate[0]),int(self.position_estimate[1]),self.blue)

            # obstcale mapping
            if self.position_estimate[0] + self.range_sensors[0] < self.map.shape[0]:
                if self.range_sensors[0] < self.thresh_obstcl:
                    self.draw9(int(self.position_estimate[0] + self.range_sensors[0]),int(self.position_estimate[1]),self.red)  # front
            if self.position_estimate[0] - self.range_sensors[1] > 0:
                if self.range_sensors[1] < self.thresh_obstcl:
                    self.draw9(int(self.position_estimate[0] - self.range_sensors[1]),int(self.position_estimate[1]),self.red) # back
            if self.position_estimate[1] + self.range_sensors[2] < self.map.shape[1]:
                if self.range_sensors[2] < self.thresh_obstcl:
                    self.draw9(int(self.position_estimate[0]),int(self.position_estimate[1] + self.range_sensors[2]),self.red)# left
            if self.position_estimate[1] - self.range_sensors[3] > 0:
                if self.range_sensors[3] < self.thresh_obstcl:
                    self.draw9(int(self.position_estimate[0]),int(self.position_estimate[1] - self.range_sensors[3]),self.red) #right

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

    def save_img(self):
        cv2.imwrite("slam.jpg", self.map)


if __name__ == '__main__':
    # initialize the low level drivers, mandatory
    cflib.crtp.init_drivers()
    # drone radio identifier
    URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E702')
    from drone import Drone

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        # creates a drone (inherits from motion commander)

        drone = Drone(scf, default_height=0.2)
        slam = Slam()

        drone.start_logs()

        drone.take_off()

        print("land")
        drone.land()
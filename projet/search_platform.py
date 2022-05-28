# Libraries python
import time
import numpy as np

# Libraries crazyflie
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper

# Libraries personnelles
from drone import Drone
from arena import Direction
from crossing_middle_zone import obstacle_detection

# container to save constant states from one pass to another
class states():
    next_segment = True
    segment = 0
    distance = 0
    start_position = 0
    line_coord = 0


def search_platform(drone:Drone, dimension_y, dimension_x, position_wall_west, height):

    # distances to travel along the x-axis and the y-axis, as defined by the arena
    DISTANCE_Y = dimension_y
    DISTANCE_X = dimension_x
    SPEED_FORWARD = 0.3
    SPEED_LATERAL = 0.3

    edge_detected = False
    
    # ======================================
    # Zigzag search pattern
    # ======================================
    
    # pattern parameters are updated with each new segment
    if states.next_segment:
        position_estimate = update_position(drone)                # Estimated position of the drone at the start of the segment
        # First move to reach the edge of the arena, is only done once
        if states.segment == 0:
            print("segment 0")
            drone.direction = Direction.LEFT                          # main direction of travel
            states.distance = position_wall_west                      # Straight distance to travel
            states.start_position = position_estimate[1]                # Starting coordinate of the segment, according to the defined direction
            states.line_coord = position_estimate[0]                    # Coordinate ("vector") of the line to follow
        # First segment, advance in x
        if states.segment == 1:
            print("segment 1")
            drone.direction = Direction.FORWARD
            states.distance = DISTANCE_X
            states.start_position = states.line_coord # crade
            states.line_coord = position_estimate[1]
        # Second segment, advance to the other side of the arena in y (to the right)
        elif states.segment == 2:
            print("segment 2")
            drone.direction = Direction.RIGHT
            states.start_position = position_estimate[1]
            states.distance = DISTANCE_Y
            states.line_coord = position_estimate[0]
        # Third segment, advance in x
        elif states.segment == 3:
            print("segment 3")
            drone.direction = Direction.FORWARD
            states.distance = DISTANCE_X
            states.start_position = states.line_coord #crade
            states.line_coord = position_estimate[1]
        # Fourth segment, advance to the other side of the arena in y (to the left)
        elif states.segment == 4:
            print("segment 4")
            drone.direction = Direction.LEFT
            states.start_position = position_estimate[1]
            states.distance = DISTANCE_Y
            states.line_coord = position_estimate[0]
            
        states.next_segment = False

    # Checks the distance, the presence of obstacles and the presence of the platform
    distance_detected = distance_detection(drone, states.distance, states.start_position, drone.direction)
    speed_x, speed_y = obstacle_detection(drone, states.line_coord, forward_speed=SPEED_FORWARD, lateral_come_back_speed=SPEED_LATERAL, direction=drone.direction)
    if drone.on_track:
        edge_detected = edge_detection(drone, fly_height=height, threshold=drone.TRESHOLD_UP)
    
    # the drone moves until the desired distance is reached
    if distance_detected:
        # allow the update of the displacement parameters according to the next segment
        states.next_segment = True
        # allows rotation of segments from 1 to 4
        states.segment = (states.segment)%4+1
    else:
        if not edge_detected:
            drone.start_linear_motion(speed_x, speed_y, 0)
        else:
            drone.stop()  
        
    return edge_detected


def update_position(drone:Drone):
    """ Retourne dans un tableau les positions du drone sur le plan (x,y) """
    position = [0,0]
    position[0] = drone.get_log('stateEstimate.x')
    position[1] = drone.get_log('stateEstimate.y')
    return position


def distance_detection(drone:Drone, distance, start_position, direction):
    """ Fonction continue pour détecter une certaine distance rectiligne parcourue """
    
    distance_detected = False

    if direction == Direction.FORWARD:
        # detect if the distance traveled is greater than the desired distance according to the starting coordinate
        if start_position + distance < drone.get_log('stateEstimate.x'):
            distance_detected = True

    if direction == Direction.BACKWARD:
        if start_position - distance > drone.get_log('stateEstimate.x'):
            distance_detected = True

    if direction == Direction.LEFT:
        if start_position + distance < drone.get_log('stateEstimate.y'):
            distance_detected = True
        
    if direction == Direction.RIGHT:
        if start_position - distance > drone.get_log('stateEstimate.y'):
            distance_detected = True
    
    return distance_detected


def edge_detection(drone:Drone, fly_height, threshold):
    """ Fonction continue pour détecter un edge, càd un changement de hauteur brusque """

    edge_detected = False
    # current flight height
    height = drone.get_log('stateEstimate.z')

    # "Circular" array of last 5 logs from estimate.z that are higher than default_height
    if height > fly_height-0.02:
        drone.zrange = np.append(drone.zrange, height)
        drone.zrange = drone.zrange[1:]

    # Detect a change in height according to the threshold = detection of the platform
    moy = np.mean(drone.zrange[:-1])

    if drone.zrange[0] != 0 and moy > fly_height-0.05 and (drone.zrange[-1] < moy - threshold or drone.zrange[-1] > moy + threshold): #detecte si on est passé au dessus de qqch (plateforme)
        print("edge found")
        edge_detected = True

    return edge_detected



if __name__ == '__main__':
    # initalise les drivers low level, obligatoire
    cflib.crtp.init_drivers()
    # identifiant radio du drone
    URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E702')

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        # crée un drone (hérite de motion commander)
        drone = Drone(scf, default_height=0.2)

        drone.start_logs()

        drone.take_off()

        edge_detected = False
        while(not edge_detected):
            drone.stop_by_hand()
            edge_detected = search_platform(drone, 0.2)
            time.sleep(0.1)
        drone.land()

        drone.stop_logs(save=False)
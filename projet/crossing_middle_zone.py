# python libraries
import time
import numpy as np

# crazyflie libraries
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper

# custom libraries
from drone import Drone
from arena import Arena, Direction

# container used to save constant states from one pass to another
class states():
    obstacle_frontal = False   # if frontal obstacle
    obstacle_lateral = False   # if lateral obstacle
    obstacle_wait = False      # to try to return after an obstacle
    default_direction = -1     # right (-1) or left (1) direction for frontal avoidance
    time_since_obstacle = 0    # time since last obstacle seen
    time_choice = 0            # the time differs depending on whether the obstacle is frontal or lateral

def go_to_middle(drone:Drone, central_line, pos_y):

        SPEED_FORWARD = 0.3
        SPEED_LATERAL = 0.3
        if pos_y > 0:
            direction = Direction.RIGHT
        else:
            direction = Direction.LEFT
        # Checks the arrival in the area of the second platform and the presence of obstacles
        arrival = aligned_plateform(drone)
        speed_x, speed_y = obstacle_detection(drone, central_line, forward_speed=SPEED_FORWARD, lateral_come_back_speed=SPEED_LATERAL, direction=direction)
            
        if not arrival: 
            drone.start_linear_motion(speed_x, speed_y, 0)            

        return arrival

def crossing_middle_zone(drone:Drone, central_line):

        SPEED_FORWARD = 0.4
        SPEED_LATERAL = 0.35
        # Checks the arrival in the area of the second platform and the presence of obstacles
        arrival = zone_P2_detection(drone)
        speed_x, speed_y = obstacle_detection(drone, central_line, forward_speed=SPEED_FORWARD, lateral_come_back_speed=SPEED_LATERAL, direction=Direction.FORWARD)
            
        if not arrival: 
            drone.start_linear_motion(speed_x, speed_y, 0)            

        return arrival

def obstacle_detection(drone:Drone, line_coord, forward_speed, lateral_come_back_speed, direction:Direction):
    """ detect obstacle and return a pair of speed (x,y) to avoid
        Param: line_position the y line to follow from drone origin """

    AVOID_DIST_FRONT = 400  # mm
    AVOID_SPEED_LAT = 0.5 
    AVOID_SPEED_COME_BACK = lateral_come_back_speed
    AVOID_TIME_COME_BACK_FRONTAL = 2 # seconds
    AVOID_TIME_COME_BACK_LATERAL = 0.7
    AVOID_SPEED_FRONT = 0.6
    FORWARD_SPEED = forward_speed
    # ======= adaptation of lateral distance ==============
    # if the drone is on the right trajectory, it will be less sensitive than if it returns to position
    if drone.on_track:
        AVOID_DIST_LAT = 90
    else:
        AVOID_DIST_LAT = 160

    # Threshold for maintaining the tracking line without oscillations
    POSITION_DIRECTION_THRESH = 0.05

    # create an array to put the 5 sensor variables
    # range [front,back,left,right,up]
    range_sensors = np.empty(5)
    position_estimate = [0, 0]

    # variables to set the lateral speed of the drone
    correction = 0
    speed_east = 0
    speed_east_lat = 0
    speed_east_front = 0

    # avoidance direction, depending on the side of the drone relative to the tracking line
    avoid_dir = states.default_direction

    # adjust sensors and variables based on the direction the drone is moving
    if direction == Direction.FORWARD: # reference
        range_sensors[0] = drone.get_log('range.front')
        range_sensors[1] = drone.get_log('range.back')
        range_sensors[2] = drone.get_log('range.left')
        range_sensors[3] = drone.get_log('range.right')
        position_estimate[0] = drone.get_log('stateEstimate.x')
        position_estimate[1] = drone.get_log('stateEstimate.y')
        RIGHT = -1
        LEFT = 1
    elif direction == Direction.LEFT:
        range_sensors[3] = drone.get_log('range.front')
        range_sensors[2] = drone.get_log('range.back')
        range_sensors[0] = drone.get_log('range.left')
        range_sensors[1] = drone.get_log('range.right')
        position_estimate[1] = drone.get_log('stateEstimate.x')
        position_estimate[0] = drone.get_log('stateEstimate.y')
        RIGHT = 1
        LEFT = -1
    elif direction == Direction.RIGHT:
        range_sensors[2] = drone.get_log('range.front')
        range_sensors[3] = drone.get_log('range.back')
        range_sensors[1] = drone.get_log('range.left')
        range_sensors[0] = drone.get_log('range.right')
        position_estimate[1] = drone.get_log('stateEstimate.x')
        position_estimate[0] = drone.get_log('stateEstimate.y')
        RIGHT = -1
        LEFT = 1
    else:
        range_sensors[1] = drone.get_log('range.front')
        range_sensors[0] = drone.get_log('range.back')
        range_sensors[3] = drone.get_log('range.left')
        range_sensors[2] = drone.get_log('range.right')
        position_estimate[0] = drone.get_log('stateEstimate.x')
        position_estimate[1] = drone.get_log('stateEstimate.y')
        RIGHT = 1
        LEFT = -1
    range_sensors[4] = drone.get_log('range.up')

    # ====== lateral avoidance =======
    if range_sensors[2] < AVOID_DIST_LAT:  # obstacle detected on the left
        states.obstacle_lateral = True
        speed_east_lat = -AVOID_SPEED_LAT
    elif range_sensors[3] < AVOID_DIST_LAT:  # obstacle detected on the right
        states.obstacle_lateral = True
        speed_east_lat = AVOID_SPEED_LAT
    else:
        states.obstacle_lateral = False
        speed_east_lat = 0


    # ====== frontal avoidance ========
    if range_sensors[0] < AVOID_DIST_FRONT:
        states.obstacle_frontal = True
        if position_estimate[1] > line_coord + 1:  # too much on the left avoids on the right
            avoid_dir = RIGHT
            states.default_direction = RIGHT
        elif position_estimate[1] < line_coord - 1:  # too much on the right avoids on the left
            avoid_dir = LEFT
            states.default_direction = LEFT

        speed_east_front = avoid_dir * AVOID_SPEED_FRONT
    else:
        speed_east_front = 0
        states.obstacle_frontal = False


    # ====== update y speed ======
    speed_east = (speed_east_lat + speed_east_front)

    if speed_east != 0:
        correction = FORWARD_SPEED/2  # correction pour aller plus lentement quand il y a des obstacles
    
    # ====== comes back on the line if no obstacle ===============
    if states.obstacle_wait == False:
        correction = 0
        if position_estimate[1] > line_coord + POSITION_DIRECTION_THRESH:
            speed_east =  RIGHT * AVOID_SPEED_COME_BACK
            drone.on_track = False
        elif position_estimate[1] < line_coord - POSITION_DIRECTION_THRESH:
            speed_east =  LEFT * AVOID_SPEED_COME_BACK
            drone.on_track = False
        else:
            speed_east = 0
            drone.on_track = True

    # ===== Delay before declaring end of obstacle to return to direction line
    if states.obstacle_lateral:
        states.time_since_obstacle = time.time_ns()
        states.obstacle_wait = True
        states.time_choice = AVOID_TIME_COME_BACK_LATERAL
    if states.obstacle_frontal:
        states.time_since_obstacle = time.time_ns()
        states.obstacle_wait = True
        states.time_choice = AVOID_TIME_COME_BACK_FRONTAL
    if time.time_ns() - states.time_since_obstacle > states.time_choice*1e9:
        states.obstacle_wait = False
        
    speed_north = FORWARD_SPEED-correction

    # adjusts the speeds of the drone in x and y according to the speeds calculated from the orientation
    if direction == Direction.FORWARD:
        speed_x, speed_y = speed_north, speed_east
    elif direction == Direction.LEFT:
        speed_x, speed_y = -speed_east, speed_north
    elif direction == Direction.RIGHT:
        speed_x, speed_y = speed_east, -speed_north
    elif direction == Direction.BACKWARD:
        speed_x, speed_y = -speed_north, -speed_east
    
    return speed_x, speed_y

def aligned_plateform(drone:Drone):
    arrival = False

    if drone.get_log('stateEstimate.y') < 0+0.25 and drone.get_log('stateEstimate.y') > 0-0.25:
        arrival = True

    return arrival

def zone_P2_detection(drone:Drone):
    """ checks when the drone arrives in the area of the arrival platform """

    arrival = False

    if drone.get_log('stateEstimate.x') >= Arena.START_ZONE_2 + 0.25:
        arrival = True

    return arrival


if __name__ == '__main__':
    # initialize the low level drivers, mandatory
    cflib.crtp.init_drivers()
    # drone radio identifier
    URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E702')

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        # creates a drone (inherits from motion commander)
        drone = Drone(scf, default_height=0.2)

        # the drone follows the line in the center of the arena
        central_line = - (Arena.ORIGIN_Y - Arena.WIDTH/2)
        # update of the default direction if there is an obstacle
        if central_line > 0:
            states.default_direction = 1
        else:
            states.default_direction = -1

        drone.start_logs()

        drone.take_off()

        crossed_middle_zone = False
        while(not crossed_middle_zone):
            drone.stop_by_hand()
            crossed_middle_zone = crossing_middle_zone(drone, central_line)
            time.sleep(0.1)

        drone.land()

        drone.stop_logs(save=False)
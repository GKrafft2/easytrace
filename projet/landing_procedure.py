# python libraries
import time
import numpy as np

# Crazyflie Libraries
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper

# Personal Libraries
from drone import Drone
from arena import Direction
from search_platform import edge_detection


def landing_procedure(drone:Drone, direction, height, search_first_edge=False):
    """ 
    Find the first edge, then move right to find the second edge
    before returning to the center with a P controller
    """
    SPEED_FORWARD = 0.25                # forward movement speed
    SPEED_LATERAL = 0.1                 # sideways movement speed
    DIST_CENTER_FRONTAL_ATTACK = 0.05   # distance to move edge forward with speed 0.04
    DIST_CENTER_LATERAL_ATTACK = 0.22   # distance to move edge left

    # Choose the orientation of the drone to land
    if direction == Direction.FORWARD: # reference
        speed1_x = SPEED_FORWARD                        
        speed1_y = 0                                    
        dist_plateform_x = DIST_CENTER_FRONTAL_ATTACK
        dist_plateform_y = 0
        speed2_x = 0
        speed2_y = SPEED_LATERAL
        half_plateform_x = 0
        half_plateform_y = DIST_CENTER_LATERAL_ATTACK

    elif direction == Direction.LEFT:
        speed1_x = 0
        speed1_y = SPEED_FORWARD
        dist_plateform_x = 0
        dist_plateform_y = DIST_CENTER_FRONTAL_ATTACK
        speed2_x = SPEED_LATERAL
        speed2_y = 0
        half_plateform_x = DIST_CENTER_LATERAL_ATTACK
        half_plateform_y = 0
        
    elif direction == Direction.RIGHT:
        speed1_x = 0
        speed1_y = -SPEED_FORWARD
        dist_plateform_x = 0
        dist_plateform_y = -DIST_CENTER_FRONTAL_ATTACK
        speed2_x = SPEED_LATERAL
        speed2_y = 0
        half_plateform_x = DIST_CENTER_LATERAL_ATTACK
        half_plateform_y = 0

    elif direction == Direction.BACKWARD:
        speed1_x = -SPEED_FORWARD
        speed1_y = 0
        dist_plateform_x = -DIST_CENTER_FRONTAL_ATTACK
        dist_plateform_y = 0
        speed2_x = 0
        speed2_y = SPEED_LATERAL
        half_plateform_x = 0
        half_plateform_y = DIST_CENTER_LATERAL_ATTACK
    
        # execution only if the function is launched individually
        # search_first_edge should be False if the function is executed after platform search
    if search_first_edge:
        # Straight movement until detection of the platform
        fly = True
        time1 = time.time_ns()
        while(fly):

            time.sleep(0.1)

            drone.start_linear_motion(speed1_x, speed1_y, 0)
            drone.stop_by_hand()
            edge_detected = edge_detection(drone, fly_height=height, threshold=drone.TRESHOLD_UP)
            # wait a second of stabilization before accepting edges
        
            if edge_detected and time.time_ns()-time1 > 1*1e9:
                print("stop home function")
                fly = False
            

    print("platforme operation")
    # move a little to the center of the platform
    drone.move_distance(distance_x_m=-dist_plateform_x, distance_y_m=-dist_plateform_y, distance_z_m=0, velocity=0.05) 
    # go back a bit
    # stabilization
    time.sleep(0.2)

    # find edge 2 of the platform
    if direction == Direction.FORWARD or direction == Direction.BACKWARD:
        position_history = drone.get_log('stateEstimate.y')
    elif direction == Direction.LEFT or direction == Direction.RIGHT:
        position_history = drone.get_log('stateEstimate.x')
    print("cherche edge 2")
    fly = True
    # if was on the edge from the start and couldn't detect the platform, turn around
    U_turn = False
    while(fly):
        drone.stop_by_hand()

        if direction == Direction.FORWARD or direction == Direction.BACKWARD:
            current_position = drone.get_log('stateEstimate.y')
        elif direction == Direction.LEFT or direction == Direction.RIGHT:
            current_position = drone.get_log('stateEstimate.x')

        # if the drone searches for the edge without finding it after 50cm,
        # it's probably that the platform was on the other side
        if current_position < position_history - 0.5:
            U_turn = True
            # set the distance the same as when going up
            half_plateform_x /= (DIST_CENTER_LATERAL_ATTACK/DIST_CENTER_FRONTAL_ATTACK)
            half_plateform_y /= (DIST_CENTER_LATERAL_ATTACK/DIST_CENTER_FRONTAL_ATTACK)

        if U_turn == True:
            drone.start_linear_motion(speed2_x, speed2_y, 0)
            edge_detected = edge_detection(drone, fly_height=height, threshold=drone.TRESHOLD_UP)
        else:
            drone.start_linear_motion(-speed2_x, -speed2_y, 0)
            edge_detected = edge_detection(drone, fly_height=height, threshold=drone.THRESHOLD_DOWN)
        
        
        if edge_detected:
            drone.stop()
            fly = False
        time.sleep(0.1)
    print("stabilisation")
    # stabilisation
    time.sleep(0.2)

    # we estimate the center of the box based on where it detects an edge
    center_x = drone.get_log('stateEstimate.x') + half_plateform_x
    center_y = drone.get_log('stateEstimate.y') + half_plateform_y
    print("go to P")
    go_to_P(drone, center_x, center_y)

    drone.land(velocity=0.07)

    landed = 1

    return landed


def go_to_P(drone:Drone, x, y):
    """ Go to function with P controller """

    error = 1
    fly = True

    position_estimate = [0,0]
    position_estimate[0] = drone.get_log('stateEstimate.x') 
    position_estimate[1] = drone.get_log('stateEstimate.y')


    print(f'Position drone x = {position_estimate[0]:.3f} y = {position_estimate[1]:.3}')
    print(f'Position box x = {x:.3f} y = {y:.3f}')

    while(fly):
        drone.stop_by_hand()
        
        position_estimate[0] = drone.get_log('stateEstimate.x') 
        position_estimate[1] = drone.get_log('stateEstimate.y')

        # try to return to the center of the box
        if error > 0.03 :
            landing_speed = -0.1
            P = 5.5
            err_x = (position_estimate[0]-x)
            err_y = (position_estimate[1]-y)
            landing_speed_x = err_x * landing_speed * P
            landing_speed_y = err_y * landing_speed * P

            drone.start_linear_motion(landing_speed_x,landing_speed_y, 0)
            error = np.sqrt(err_x**2+err_y**2)

        # if the error is small enough we land
        else:
            print(f'pos drone x = {position_estimate[0]:.3f} y = {position_estimate[1]:.3}')
            print('Goal achieved ')
            
            fly = False
            drone.stop()

        time.sleep(0.1)


if __name__ == '__main__':

    # initalise les drivers low level, obligatoire
    cflib.crtp.init_drivers()
    # identifiant radio du drone
    URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E702')

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:

        # crée un drone (hérite de motion commander)
        drone = Drone(scf, default_height=0.2)
        
        drone.start_logs()

        drone.take_off(0.2)

        landing_procedure(drone, direction=Direction.BACKWARD, height=0.2, search_first_edge=True)
       
        drone.stop_logs()



        
       
        



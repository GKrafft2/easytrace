
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
from arena import arena
from p_crossing import avoid
from p_crossing import Direction
from p_land_platform import landing_procedure, edge_detection


def main_search_platform(drone:Drone):
    
    #search zone
    CORNER = [[0,0.5],[0,1]]

    # variable pour la position actuelle
    position_estimate = [0, 0]
    position_estimate[0] = drone.get_log('stateEstimate.x')
    position_estimate[1] = drone.get_log('stateEstimate.y')

    # variable qui enregistre la position au début du segment
    start_position = [0, 0]
    start_position[0] = drone.get_log('stateEstimate.x')
    start_position[1] = drone.get_log('stateEstimate.y')
    OFFSET_X = 0.3

    # distances à parcourir selon l'axe x et l'axe y
    distance_y = arena.WIDTH
    distance_x = 0.3

    landed = False

    # premier déplacement pour rejoindre le bord de l'arène
    landed = move(drone, arena.LIM_WEST - position_estimate[1], position_estimate[0], start_position, Direction.LEFT)
    if landed:
        fly = False
    else:
        fly = True

    while(fly):
        position_estimate[0] = drone.get_log('stateEstimate.x')
        position_estimate[1] = drone.get_log('stateEstimate.y')
        
        # attéri si fin de limitation
        if (position_estimate[0] + OFFSET_X) > CORNER[0][1]:
            fly = 0

        # avance de 10cm
        landed = move(drone, distance_x, position_estimate[1], start_position, Direction.FORWARD)
        if landed:
            fly = 0
        # va au bord droite
        start_position[0] = drone.get_log('stateEstimate.x')
        start_position[1] = drone.get_log('stateEstimate.y')
        position_estimate[0] = drone.get_log('stateEstimate.x')
        position_estimate[1] = drone.get_log('stateEstimate.y')
        print('start position', start_position)
        landed = move(drone, distance_y, position_estimate[0], start_position, Direction.RIGHT)
        if landed:
            fly = 0
        # avance de 10cm
        position_estimate[1] = drone.get_log('stateEstimate.y')
        landed = move(drone, distance_x, position_estimate[1], start_position, Direction.FORWARD)
        if landed:
            fly = 0
        
        # va au bord gauche
        start_position[0] = drone.get_log('stateEstimate.x')
        start_position[1] = drone.get_log('stateEstimate.y')
        position_estimate[0] = drone.get_log('stateEstimate.x')
        position_estimate[1] = drone.get_log('stateEstimate.y')
        landed = move(drone, distance_y, position_estimate[0], start_position, Direction.LEFT)
        if landed:
            fly = 0

def move(drone:Drone, distance, line_coord, start_position, direction):
    # direction_x: avant=1, arrière=-1
    # direction_y: droite=-1, gauche=1

    position_estimate = [0, 0]

    reach = False
    edge_detected = False

    while(not reach and not edge_detected):
            
        drone.stop_by_hand()
        position_estimate[0] = drone.get_log('stateEstimate.x')
        position_estimate[1] = drone.get_log('stateEstimate.y')
        # print(f'x = {position_estimate[0]:.2f}  y = {position_estimate[1]:.2f} {(start_position[1] + distance):.2f}')

        # drone.update_slam()
        if direction == Direction.FORWARD:
            if abs(start_position[0] + distance)<position_estimate[0]:
                reach = 1
                drone.stop()
            speed_x, speed_y = avoid(drone, line_coord, Direction.FORWARD)
            # empêche la détection de la plateform par le côté latéral du drone
            # on ne détecte que quand le drone va droit de manière stable
            if drone.on_track:
                edge_detected, _, _ = edge_detection(drone, height=0.2, threshold=0.013)
            drone.start_linear_motion(speed_x, speed_y, 0)
        if direction == Direction.BACKWARD:
            if abs(start_position[0] - distance)>position_estimate[0]:
                reach = 1
                drone.stop()
            speed_x, speed_y = avoid(drone, line_coord, Direction.BACKWARD)
            drone.start_linear_motion(speed_x, speed_y, 0)
        if direction == Direction.LEFT:
            if (start_position[1] + distance)<position_estimate[1]:
                reach = 1
                drone.stop()
            print('left')
            print(f'x = {position_estimate[0]:.2f}  y = {position_estimate[1]:.2f} {(start_position[1] + distance):.2f}')
            speed_x, speed_y = avoid(drone, line_coord, Direction.LEFT)
            if drone.on_track:
                edge_detected, _, _ = edge_detection(drone, height=0.2, threshold=0.013)
            drone.start_linear_motion(speed_x, speed_y, 0)
        if direction == Direction.RIGHT:
            print('right')
            if (start_position[1] - distance)>position_estimate[1]:
                reach = 1
                drone.stop()
            print(f'x = {position_estimate[0]:.2f}  y = {position_estimate[1]:.2f} {(start_position[1] - distance):.2f}')
            speed_x, speed_y = avoid(drone, line_coord, Direction.RIGHT)
            if drone.on_track:
                edge_detected, _, _ = edge_detection(drone, height=0.2, threshold=0.013)
            drone.start_linear_motion(speed_x, speed_y, 0)
        
        time.sleep(0.1)
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

        # fly_while_avoid(drone)
        main_search_platform(drone)

        print("land")
        drone.land()

        # drone.slam.slam_hold()

        drone.stop_logs(save=False)
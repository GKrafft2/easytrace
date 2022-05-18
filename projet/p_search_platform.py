
# Libraries python
from platform import platform
import time
import numpy as np

# Libraries crazyflie
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper

# Libraries personnelles
from drone import Drone
from arena import Arena, Platform
from p_crossing import avoid
from p_crossing import Direction
from p_land_platform import landing_procedure, edge_detection


def main_search_platform(drone:Drone):
    
    #Zone de recherche
    CORNER = [[0,0.5],[0,1]]

    # variable pour la position actuelle
    position_estimate = update_position(drone)

    # variable qui enregistre la position au début du segment
    start_position = update_position(drone)
    OFFSET_X = 0.3

    # distances à parcourir selon l'axe x et l'axe y
    distance_y = Arena.WIDTH
    distance_x = Platform.SIZE

    edge_detected = False

    # premier déplacement pour rejoindre le bord de l'arène
    edge_detected = move(drone, Arena.LIM_WEST - position_estimate[1], position_estimate[0], start_position, Direction.LEFT)
    if edge_detected:
        fly = False
    else:
        fly = True

    # Pattern de recherche en forme de zigzag
    while(fly):
        
        # attéri si en dehors de l'arène
        if (position_estimate[0] + OFFSET_X) > CORNER[0][1]:
            fly = False
            break

        # Premier segment, avance de 10cm en x
        position_estimate = update_position(drone)
        drone.direction = Direction.FORWARD
        edge_detected = move(drone, distance_x, position_estimate[1], start_position, Direction.FORWARD)
        if edge_detected:
            fly = False
            break

        # Deuxième segment, avance de l'autre côté de l'arène en y (vers la droite)
        position_estimate = update_position(drone)
        start_position = update_position(drone)
        print('start position', start_position)
        drone.direction = Direction.RIGHT
        edge_detected = move(drone, distance_y, position_estimate[0], start_position, Direction.RIGHT)
        if edge_detected:
            fly = False
            break

        # Troisième segment, avance de 10cm en x
        position_estimate = update_position(drone)
        drone.direction = Direction.FORWARD
        edge_detected = move(drone, distance_x, position_estimate[1], start_position, Direction.FORWARD)
        if edge_detected:
            fly = False
            break
    
        # Quatrième segment, avance de l'autre côté de l'arène en y (vers la gauche)
        start_position = update_position(drone)
        position_estimate = update_position(drone)
        drone.direction = Direction.LEFT
        edge_detected = move(drone, distance_y, position_estimate[0], start_position, Direction.LEFT)
        if edge_detected:
            fly = False
            break

    drone.stop()

def update_position(drone:Drone):
    position = [0,0]
    position[0] = drone.get_log('stateEstimate.x')
    position[1] = drone.get_log('stateEstimate.y')
    return position

def move(drone:Drone, distance, line_coord, start_position, direction):
    """ Fonction de déplacement rectiligne sur une certaine distance tout en détectant
        les obstacles et un changement de hauteur
    """
    position_estimate = [0, 0]

    reach_distance = False
    edge_detected = False

    while(not reach_distance and not edge_detected):
            
        drone.stop_by_hand()
        position_estimate[0] = drone.get_log('stateEstimate.x')
        position_estimate[1] = drone.get_log('stateEstimate.y')

        # drone.update_slam()
        if direction == Direction.FORWARD:
            if abs(start_position[0] + distance)<position_estimate[0]:
                reach_distance = True
            speed_x, speed_y = avoid(drone, line_coord, Direction.FORWARD)
            # print(f'x = {position_estimate[0]:.2f}  y = {position_estimate[1]:.2f} {(start_position[1] + distance):.2f}')

        if direction == Direction.BACKWARD:
            if abs(start_position[0] - distance)>position_estimate[0]:
                reach_distance = True
            speed_x, speed_y = avoid(drone, line_coord, Direction.BACKWARD)

        if direction == Direction.LEFT:
            if (start_position[1] + distance)<position_estimate[1]:
                reach_distance = True
            speed_x, speed_y = avoid(drone, line_coord, Direction.LEFT)
           
        if direction == Direction.RIGHT:
            if (start_position[1] - distance)>position_estimate[1]:
                reach_distance = True
            speed_x, speed_y = avoid(drone, line_coord, Direction.RIGHT)
        
        if reach_distance:
            drone.stop()
        else:
            drone.start_linear_motion(speed_x, speed_y, 0)

        # empêche la détection de la plateform par le côté latéral du drone
        # on ne détecte que quand le drone va droit de manière stable
        if drone.on_track:
            edge_detected, _, _ = edge_detection(drone, height=0.2, threshold=0.013)

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

        main_search_platform(drone)

        drone.land()


        drone.stop_logs(save=False)
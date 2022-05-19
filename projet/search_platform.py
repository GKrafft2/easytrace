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
from arena import Arena, Platform, Direction
from crossing_middle_zone import obstacle_detection

def search_platform(drone:Drone, height):

    # distances à parcourir selon l'axe x et l'axe y
    DISTANCE_Y = Arena.WIDTH
    DISTANCE_X = Platform.SIZE

    edge_detected = False
    
    # Pattern de recherche en forme de zigzag
    
    # les paramètres du pattern sont actualisé à chaque nouveau segment
    if drone.next_segment:
        # premier déplacement pour rejoindre le bord de l'arène, n'est effectué qu'une fois
        if drone.segment == 0:
            drone.direction = Direction.LEFT
            position_estimate = update_position(drone)
            distance = Arena.LIM_WEST - position_estimate[1]
            start_position = position_estimate[1]
            line_coord = position_estimate[0]
        # Premier segment, avance de 10cm en x
        if drone.segment == 1:
            drone.direction = Direction.FORWARD
            position_estimate = update_position(drone)
            distance = DISTANCE_X
            line_coord = position_estimate[1]
        # Deuxième segment, avance de l'autre côté de l'arène en y (vers la droite)
        elif drone.segment == 2:
            drone.direction = Direction.RIGHT
            position_estimate = update_position(drone)
            start_position = position_estimate[1]
            distance = DISTANCE_Y
            line_coord = position_estimate[0]
        # Troisième segment, avance de 10cm en x
        elif drone.segment == 3:
            drone.direction = Direction.FORWARD
            position_estimate = update_position(drone)
            distance = DISTANCE_X
            line_coord = position_estimate[1]
        # Quatrième segment, avance de l'autre côté de l'arène en y (vers la gauche)
        elif drone.segment == 4:
            drone.direction = Direction.LEFT
            position_estimate = update_position(drone)
            start_position = position_estimate[1]
            distance = DISTANCE_Y
            line_coord = position_estimate[0]
        
        drone.next_segment = False
        # permet une rotation de 1 à 4
        drone.segment = (drone.segment+1)%4+1

    # quitte si en dehors de l'arène
    if (position_estimate[0] + DISTANCE_X) > Arena.REGION_LENGTH:
        print("En dehors des limites")
        return True

    # Vérifie la distance, la présence d'obstacles et la présence de la plateforme
    distance_detected = distance_detection(drone, distance, start_position, drone.direction)
    speed_x, speed_y = obstacle_detection(drone, line_coord, drone.direction)
    if drone.on_track:
        edge_detected = edge_detection(drone, height=height, threshold=0.013)
    
    # le drone bouge tant que la distance souhaitée n'est pas atteinte
    if distance_detected:
        drone.stop()
        drone.next_segment = True
    else:
        drone.start_linear_motion(speed_x, speed_y, 0)
        
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


def edge_detection(drone:Drone, height, threshold):
    """ Fonction continue pour détecter un edge, càd un changement de hauteur brusque """

    edge_detected = False

    height_estimate = drone.get_log('stateEstimate.z')

    # Tableau "circulaire" des 5 derniers logs de estimate.z qui sont plus élevé que default_height
    if height_estimate > height-0.02:
        drone.zrange = np.append(drone.zrange, height_estimate)
        drone.zrange = drone.zrange[1:]
        # print(drone.zrange)

    # Détecte un changement de hauteur selon le threshold = détection de la plateforme
    moy = np.mean(drone.zrange[:-1])
    if moy > height-0.05 and (drone.zrange[-1] < moy - threshold or drone.zrange[-1] > moy + threshold): #detecte si on est passé au dessus de qqch (plateforme)
        print("edge found")
        edge_detected = True
        drone.stop()

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
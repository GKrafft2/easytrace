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

# container pour sauvegarder les états constant d'un passage à l'autre
class states():
    next_segment = True
    segment = 0
    distance = 0
    start_position = 0
    line_coord = 0


def search_platform(drone:Drone, dimension_y, dimension_x, position_wall_west, height):

    # distances à parcourir selon l'axe x et l'axe y, comme définit par l'arène
    DISTANCE_Y = dimension_y
    DISTANCE_X = dimension_x
    SPEED_FORWARD = 0.3
    SPEED_LATERAL = 0.3

    edge_detected = False
    
    # =======================================
    # Pattern de recherche en forme de zigzag
    # =======================================
    
    # les paramètres du pattern sont actualisé à chaque nouveau segment
    if states.next_segment:
        position_estimate = update_position(drone)                # Position estimée du drone au début du segment
        # Premier déplacement pour rejoindre le bord de l'arène, n'est effectué qu'une fois
        if states.segment == 0:
            print("segment 0")
            drone.direction = Direction.LEFT                          # direction principale de déplacement
            states.distance = position_wall_west                      # Distance rectiligne à parcourir
            states.start_position = position_estimate[1]               # Coordonnée de départ du segment, selon la direction définie
            states.line_coord = position_estimate[0]                   # Coordonnée ("vecteur") de la ligne à suivre
        # Premier segment, avance en x
        if states.segment == 1:
            print("segment 1")
            drone.direction = Direction.FORWARD
            states.distance = DISTANCE_X
            states.start_position = states.line_coord # crade
            states.line_coord = position_estimate[1]
        # Deuxième segment, avance de l'autre côté de l'arène en y (vers la droite)
        elif states.segment == 2:
            print("segment 2")
            drone.direction = Direction.RIGHT
            states.start_position = position_estimate[1]
            states.distance = DISTANCE_Y
            states.line_coord = position_estimate[0]
        # Troisième segment, avance en x
        elif states.segment == 3:
            print("segment 3")
            drone.direction = Direction.FORWARD
            states.distance = DISTANCE_X
            states.start_position = states.line_coord #crade
            states.line_coord = position_estimate[1]
        # Quatrième segment, avance de l'autre côté de l'arène en y (vers la gauche)
        elif states.segment == 4:
            print("segment 4")
            drone.direction = Direction.LEFT
            states.start_position = position_estimate[1]
            states.distance = DISTANCE_Y
            states.line_coord = position_estimate[0]
            
        states.next_segment = False

        # # quitte si en dehors de l'arène
        # if (position_estimate[0]) > Arena.LENGTH - Arena.ORIGIN_X:
        #     print("En dehors des limites")
        #     return True

    # print(f'distance = {states.distance:.2f} start')

    # Vérifie la distance, la présence d'obstacles et la présence de la plateforme
    distance_detected = distance_detection(drone, states.distance, states.start_position, drone.direction)
    speed_x, speed_y = obstacle_detection(drone, states.line_coord, forward_speed=SPEED_FORWARD, lateral_come_back_speed=SPEED_LATERAL, direction=drone.direction)
    if drone.on_track:
        edge_detected = edge_detection(drone, fly_height=height, threshold=drone.TRESHOLD_UP)
    
    # le drone bouge tant que la distance souhaitée n'est pas atteinte
    if distance_detected:
        # drone.stop()
        # permet l'update des paramètres de déplacement selon le prochain segment
        states.next_segment = True
        # permet une rotation des segments de 1 à 4
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
        # détecte si la distance parcourue est supérieur à la distance souhaitée selon la coordonnée de départ
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
    # hauteur de vol actuelle
    height = drone.get_log('stateEstimate.z')

    # Tableau "circulaire" des 5 derniers logs de estimate.z qui sont plus élevé que default_height
    if height > fly_height-0.02:
        drone.zrange = np.append(drone.zrange, height)
        drone.zrange = drone.zrange[1:]
        # print(drone.zrange)

    # Détecte un changement de hauteur selon le threshold = détection de la plateforme
    moy = np.mean(drone.zrange[:-1])

    # print(f"height = {height:.3f} moy = {moy:.3f}")

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
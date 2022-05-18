
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

def edge_detection(drone:Drone, height, threshold):

    # Alias pour les logs de drone estimate.x estimate.y et estimate.z
    # Permet de garder les mêmes valeurs durant un passage complet de boucle
    # Un appel direct aux logs peut être actualisé entre temps
    position_estimate = [0, 0, 0, 0]
    edge_detected = False

    position_estimate[0] = drone.get_log('stateEstimate.x') 
    position_estimate[1] = drone.get_log('stateEstimate.y')
    position_estimate[2] = drone.get_log('stateEstimate.z')
    position_estimate[3] = drone.get_log('range.zrange')

    # Tableau "circulaire" des 5 derniers logs de estimate.z qui sont plus élevé que default_height
    if position_estimate[2] > height-0.02:
        drone.zrange = np.append(drone.zrange, position_estimate[2])
        drone.zrange = drone.zrange[1:]
        print(drone.zrange)

    # Détecte un changement de hauteur selon le threshold = détection de la plateforme
    THRESH = threshold
    moy = np.mean(drone.zrange[:-1])
    # print(moy-drone.zrange[-1])
    if moy > height-0.05 and (drone.zrange[-1] < moy - THRESH or drone.zrange[-1] > moy + THRESH): #detecte si on est passé au dessus de qqch (plateforme)
        #le mode landing est activé
        print("edge found") #passe la main au landing
        edge_detected = True
        drone.stop()

    # retourne la position position détectée
    return edge_detected, position_estimate[0], position_estimate[1]

def main_search_platform(drone:Drone):
    
    #search zone
    CORNER = [[0,0.5],[0,1]]

    position_estimate = [0, 0]
    position_estimate[0] = drone.get_log('stateEstimate.x')
    position_estimate[1] = drone.get_log('stateEstimate.y')

    start_position = [0, 0]
    start_position[0] = drone.get_log('stateEstimate.x')
    start_position[1] = drone.get_log('stateEstimate.y')
    OFFSET_X = 0.3
    fly = 1

    distance_y = 1.5
    distance_x = 0.3
    # drone.height_cmd = drone.get_log('stateEstimate.z')
    drone.height_cmd = 0.4
    edge_detected = 0

    edge_detected = move(drone, arena.LIM_WEST - position_estimate[1], position_estimate[0], position_estimate, direction_y=1)
    if edge_detected:
        fly = 0

    while(fly):
        position_estimate[0] = drone.get_log('stateEstimate.x')
        position_estimate[1] = drone.get_log('stateEstimate.y')
        
        # attéri si fin de limitation
        if (position_estimate[0] + OFFSET_X) > CORNER[0][1]:
            fly = 0

        # avance de 10cm
        edge_detected = move(drone, distance_x, position_estimate[1], start_position, direction_x=1)
        if edge_detected:
            fly = 0
        # va au bord droite
        start_position[0] = drone.get_log('stateEstimate.x')
        start_position[1] = drone.get_log('stateEstimate.y')
        position_estimate[0] = drone.get_log('stateEstimate.x')
        edge_detected = move(drone, distance_y, position_estimate[0], direction_y=-1)
        if edge_detected:
            fly = 0
        # avance de 10cm
        position_estimate[1] = drone.get_log('stateEstimate.y')
        edge_detected = move(drone, distance_x, position_estimate[1], start_position, direction_x=1)
        if edge_detected:
            fly = 0
        
        # va au bord gauche
        position_estimate[0] = drone.get_log('stateEstimate.x')
        edge_detected = move(drone, distance_y, position_estimate[0], direction_y=1)
        if edge_detected:
            fly = 0

def move(drone:Drone, distance, line_position, start_position=[0, 0], direction_x=0, direction_y=0, speed_x=0.2, speed_y=0.2):
    # direction_x: avant=1, arrière=-1
    # direction_y: droite=-1, gauche=1
    position_estimate = [0, 0]
    position_estimate[0] = drone.get_log('stateEstimate.x')
    position_estimate[1] = drone.get_log('stateEstimate.y')

    print(start_position,'start position')
    print(direction_x,direction_y)

    reach = 0
    edge_detected = 0
    drone.start_linear_motion(direction_x*speed_x, direction_y*speed_y, 0)
    while(not reach and not edge_detected):
        # empêche la détection de la plateform par le côté latéral du drone
        # on ne détecte que quand le drone va droit de manière stable
        if drone.on_track:
            edge_detected, _, _ = edge_detection(drone)
            
        drone.stop_by_hand()
        position_estimate[0] = drone.get_log('stateEstimate.x')
        position_estimate[1] = drone.get_log('stateEstimate.y')
        # print(f'x = {position_estimate[0]:.2f}  y = {position_estimate[1]:.2f} {(start_position[1] + distance):.2f}')

        # drone.update_slam()
        if not direction_y and direction_x == 1:
            if abs(start_position[0] + distance)<position_estimate[0]:
                reach = 1
                drone.stop()
            speed_x, speed_y = avoid(drone, line_position, Direction.FORWARD)
            drone.start_linear_motion(speed_x, speed_y, 0)
        if not direction_y and direction_x == -1:
            if abs(start_position[0] + direction_x*distance)>position_estimate[0]:
                reach = 1
                drone.stop()
            speed_x, speed_y = avoid(drone, line_position, Direction.BACKWARD)
            drone.start_linear_motion(speed_x, speed_y, 0)
        if not direction_x and direction_y == 1:
            if abs(start_position[1] + distance)<position_estimate[1]:
                reach = 1
                drone.stop()
            speed_x, speed_y = avoid(drone, line_position, Direction.LEFT)
            drone.start_linear_motion(speed_x, speed_y, 0)
        if not direction_x and direction_y == -1:
            if abs(start_position[1] + direction_y*distance)>position_estimate[1]:
                reach = 1
                drone.stop()
            print(f'x = {position_estimate[0]:.2f}  y = {position_estimate[1]:.2f} {(start_position[1] + distance):.2f}')
            speed_x, speed_y = avoid(drone, line_position, Direction.RIGHT)
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
        drone = Drone(scf, default_height=0.4)

        drone.start_logs()

        drone.take_off()

        # fly_while_avoid(drone)
        main_search_platform(drone)

        print("land")
        drone.land()

        # drone.slam.slam_hold()

        drone.stop_logs(save=False)
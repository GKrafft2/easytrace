from enum import Enum

# Libraries python
import time
import datetime as dt
from threading import Event
import numpy as np

# Libraries crazyflie
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper

# Libraries personnelles
from drone import Drone
from arena import arena, platform
from p_crossing import Direction


# class Direction(Enum):
    # FORWARD = 0
    # RIGHT = 1
    # LEFT = 2
    # BACKWARD = 3


def go_to_P(drone:Drone, x, y):
    print("METHOD : land_on_platform")

    error = 1
    fly = True

    #screenshot de la pos au moment ou il detecte la boite
    position_estimate = [0,0]
    position_estimate[0] = drone.get_log('stateEstimate.x') 
    position_estimate[1] = drone.get_log('stateEstimate.y')


    print(f'pos drone x = {position_estimate[0]:.3f} y = {position_estimate[1]:.3}')
    print(f'pos boite x = {x:.3f} y = {y:.3f}')
    print(f'Platform start x = {platform.x_start:.3f} y = {platform.y_start:.3f}')

    while(fly):

        position_estimate[0] = drone.get_log('stateEstimate.x') 
        position_estimate[1] = drone.get_log('stateEstimate.y')
        # print(f'error {error}')
        # essaye de revenir au centre de la boite
        if error > 0.02 :
            landing_speed = -0.1
            P = 5.5
            err_x = (position_estimate[0]-x)
            err_y = (position_estimate[1]-y)
            landing_speed_x = err_x * landing_speed * P
            landing_speed_y = err_y * landing_speed * P

            # anti-windup
            # if landing_speed_x > 0.2:
            #     landing_speed_x = 0.2
            # if landing_speed_y > 0.2:
            #     landing_speed_y = 0.2
            # print(landing_speed_x, landing_speed_y)

            drone.start_linear_motion(landing_speed_x,landing_speed_y, 0)
            error = np.sqrt(err_x**2+err_y**2)

        # si l erreur est suffisamment petite on atterit
        else:
            print(f'pos drone x = {position_estimate[0]:.3f} y = {position_estimate[1]:.3}')
            print('Goal achieved ')
            
            fly = False
            drone.stop() #  nécessaire si on a déja un drone.land() ?

        time.sleep(0.1)

        # print(f'x = {position_estimate[0]:.2f} y = {position_estimate[1]:.2f}')

    time.sleep(0.1)

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
        # print(drone.zrange)

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

def procedure1(drone:Drone, direction):
    """ Cherche la première arête, puis se déplace vers la droite pour chercher la seconde arrête"""
    speed_forward = 0.2
    speed_latera = 0.1

    if direction == Direction.FORWARD: # référence
        #peut-etre enlever quand utiliser avec search
        speed1_x = speed_forward
        speed1_y = 0

        dist_plateform_x = 0.12
        dist_plateform_y = 0

        speed2_x = 0
        speed2_y = speed_latera

        half_plateform_x = 0
        half_plateform_y = 0.25
    elif direction == Direction.LEFT:
        #peut-etre enlever quand utiliser avec search
        speed1_x = 0
        speed1_y = speed_forward

        dist_plateform_x = 0
        dist_plateform_y = 0.12

        speed2_x = speed_latera
        speed2_y = 0

        half_plateform_x = 0.25
        half_plateform_y = 0
    elif direction == Direction.RIGHT:
        #peut-etre enlever quand utiliser avec search
        speed1_x = 0
        speed1_y = -speed_forward

        dist_plateform_x = 0
        dist_plateform_y = -0.12

        speed2_x = speed_latera
        speed2_y = 0

        half_plateform_x = 0.25
        half_plateform_y = 0
    
    
    # cherche le début de la plateforme
    # fly = True

    edge_detected, _, _ = edge_detection(drone, height=0.2, threshold=0.013)
    if not edge_detected:
        return

    # while(fly):
    #     drone.start_linear_motion(speed1_x, speed1_y, 0)
    #     drone.stop_by_hand()
    #     edge_detected, _, _ = edge_detection(drone, height=0.2, threshold=0.013)
    #     # attends une seconde de stabilisation avant d'accepter les edges
    
    #     if edge_detected and time.time_ns()-time1 > 1*1e9:
    #         drone.stop()
    #         fly = False
    #     time.sleep(0.1)

    # time1 = time.time_ns()
    # if time.time_ns()-time1 > 1*1e9:
    #     drone.stop()
    drone.stop
    time.sleep(0.1)
    print("out")
    time.sleep(1)
    # avance un peu au centre de la plateforme
    drone.move_distance(dist_plateform_x, dist_plateform_y, 0, 0.1) 
    time.sleep(2)

    # cherche le bord 2 de la plateforme
    fly = True
    if direction == Direction.FORWARD:
        position_history = drone.get_log('stateEstimate.y')
    elif direction == Direction.LEFT or direction == Direction.RIGHT:
        position_history = drone.get_log('stateEstimate.x')

    while(fly):
        if direction == Direction.FORWARD:
            current_position = drone.get_log('stateEstimate.y')
        elif direction == Direction.LEFT or direction == Direction.RIGHT:
            current_position = drone.get_log('stateEstimate.x')

        if current_position < position_history - 0.5:
            drone.start_linear_motion(speed2_x, speed2_y, 0)
            position_history = -100
            print('trop loin')
            print(current_position, 'history', position_history)
        else:
            drone.start_linear_motion(-speed2_x, -speed2_y, 0)
            print('bien')
            print(current_position)
        drone.stop_by_hand()
        edge_detected, _, y = edge_detection(drone, height=0.2, threshold=0.03)
        # attends une seconde de stabilisation avant d'accepter les edges
        if edge_detected:
            drone.stop()
            fly = False
        time.sleep(0.1)
    time.sleep(1)

    # on estime le centre de la box en fonction de là où il detecte un edge
    center_x = drone.get_log('stateEstimate.x') + half_plateform_x
    center_y = drone.get_log('stateEstimate.y') + half_plateform_y

    time.sleep(1)

    go_to_P(drone, center_x, center_y)

    drone.land()
    landed = 1
    return landed

def main_land_platform(drone:Drone):

    # stabilise correctement 
    drone.take_off()
    time.sleep(1)

    # avance tout droit jusqu'à la boite
    platform.x_start, platform.y_start = search_edge_infinite(drone, speed_x=0.25, speed_y=0)
    print(f'Début plateforme x = {platform.x_start:.4f}, y = {platform.y_start:.4f}')
    # avance encore tout en continuant de détecter les edges
    edge_detected, edge_x_position, edge_y_position = search_edge_timed(drone, speed_x=0.2, speed_y=0, distance=0.25)
    # stop le drone avec petit coup en arrière pour ne pas drifter
    # drone.stop_brutal()

    drone.stop()
    # si aucun edge n'est détecté, ingore les variables de second edge
    # if(edge_detected):
    #     print(f'Edge détecté {edge_x_position:.4f} {edge_y_position:.4f}')
    # else:
    #     edge_x_position = platform.x_start
    #     edge_y_position = platform.y_start

    # y_difference = edge_y_position - platform.y_start
    # direction_moved = landing_level_1(y_difference)
    end = False
    while(not end):
        # landing_level_2(direction_moved)
        landing_level_2(0)
        detected_sensor = landing_level_3()
        if detected_sensor == '':
            end = True
        else:
            direction_moved = 0

    
def landing_level_2(direction_moved):
    land = False
    # repète level 2 en boucle (il faut se poser sans dévier)
    while(not land):

        time.sleep(1)

        # ==== LEVEL 2 =======
        # le drone descend et détecte ensuite s'il dévie en y durant la descente
        # une descente sur un sol non planaire cause des dévaitions.
        drone.up(-0.25, 0.2)
        
        history = 200
        y_estimate = drone.get_log('stateEstimate.y', history, array=True)
        t_array = np.linspace(0, history, history)
        a, _ = np.polyfit(t_array, y_estimate, 1)
        print(f'ap = {a:.4e}')

        distance = 0.18
        if a > 1e-4 and (direction_moved == 1 or direction_moved == 0):
            print("going more right")
            drone.up(0.25, 0.4)
            time.sleep(1)
            drone.right(distance, 0.2)
        elif a < -1e-4 and (direction_moved == -1 or direction_moved == 0):
            print("going more left")
            drone.up(0.25, 0.4)
            time.sleep(1)
            drone.left(distance, 0.2)
        else:
            print("going landing")
            land = True

    drone.land()

    time.sleep(1)
    print("landed")

def landing_level_3():
    # ==== LEVEL 3 ======
    # détecte s'il n'y a pas de mur une fois atteri, sinon redécolle
    
    sensors = ['range.right', 'range.left', 'range.front', 'range.back']
    detected_sensor = ''
    for sensor in sensors:
        # si plus proche que 
        if drone.get_log(sensor) < 150:
            print("Not on platform")
            print(f'Sensor {sensor} detected object at {drone.get_log(sensor)/10} mm')
            detected_sensor = sensor

    if detected_sensor != '':
        drone.take_off(0.4)
        distance = 0.3
        if detected_sensor == 'range.right':
            drone.right(distance)
        elif detected_sensor == 'range.left':
            drone.left(distance)
        elif detected_sensor == 'range.forward':
            drone.forward(distance)
        elif detected_sensor == 'range.back':
            drone.back(distance)

        drone.stop()
        time.sleep(1)

    return detected_sensor



if __name__ == '__main__':

    # initalise les drivers low level, obligatoire
    cflib.crtp.init_drivers()
    # identifiant radio du drone
    URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E702')

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:

        # crée un drone (hérite de motion commander)
        drone = Drone(scf, default_height=0.4)
        
        drone.start_logs()
        # time.sleep(0.5)
        # while(True):
        #     print(drone.get_log('stateEstimate.y'))

        drone.take_off(0.2)
        # main_land_platform(drone)
        procedure1(drone, Direction.FORWARD)
        drone.land
        # drone.take_off(0.4)
        # # coin 1
        # drone.left(0.15, 0.2)
        # time.sleep(2)
        # drone.down(0.25, 0.1)
        # time.sleep(4)
        # # milieu
        # drone.up(0.25, 0.3)
        # time.sleep(1)
        # drone.left(0.25, 0.2)
        # time.sleep(1)
        # drone.down(0.25, 0.1)
        # time.sleep(4)
        # #coin 2
        # drone.up(0.25, 0.3)
        # drone.left(0.2, 0.2)
        # time.sleep(1)
        # drone.down(0.25, 0.1)
        # time.sleep(4)
        # drone.up(0.25, 0.3)
        

        drone.land()
        drone.stop_logs()

        # drone.go_to(x=1, y=-0.3, z=1.5)


        
       
        



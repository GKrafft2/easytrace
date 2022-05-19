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
from search_platform import edge_detection


def landing_procedure(drone:Drone, direction, height, search_first_edge=False):
    """ 
        Cherche la première arête, puis se déplace vers la droite pour chercher la seconde arrête
        avant de retourner au centre avec un controleur P
    """
    
    SPEED_FORWARD = 0.2                 # vitesse de déplacement vers l'avant
    SPEED_LATERAL = 0.1                 # vitesse de déplacement sur les côtés
    DIST_CENTER_FRONTAL_ATTACK = 0.12   # distance de déplacement du bord vers l'avant
    DIST_CENTER_LATERAL_ATTACK = 0.25   # distance de déplacement du bord vers la gauche

    # Choisi l'orientation du drone pour attérir
    if direction == Direction.FORWARD: # référence
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
    
    # exécution uniquement si la fonction est lancée individuellement
    # search_first_edge doit être False si la foncion est exécutée après la recherche de plateforme
    if search_first_edge:
        # Déplacement rectiligne jusqu'à la détection de la plateforme
        fly = True
        while(fly):
            drone.start_linear_motion(speed1_x, speed1_y, 0)
            drone.stop_by_hand()
            edge_detected = edge_detection(drone, fly_height=height, threshold=0.013)
            # attends une seconde de stabilisation avant d'accepter les edges
        
            if edge_detected and time.time_ns()-time1 > 1*1e9:
                drone.stop()
                fly = False
            time.sleep(0.1)

            time1 = time.time_ns()
            if time.time_ns()-time1 > 1*1e9:
                drone.stop()

    # avance un peu au centre de la plateforme
    drone.move_distance(dist_plateform_x, dist_plateform_y, 0, velocity=0.1) 
    # stabilisation
    time.sleep(2)

    # cherche le bord 2 de la plateforme
    if direction == Direction.FORWARD:
        position_history = drone.get_log('stateEstimate.y')
    elif direction == Direction.LEFT or direction == Direction.RIGHT:
        position_history = drone.get_log('stateEstimate.x')

    fly = True
    while(fly):
        drone.stop_by_hand()

        if direction == Direction.FORWARD:
            current_position = drone.get_log('stateEstimate.y')
        elif direction == Direction.LEFT or direction == Direction.RIGHT:
            current_position = drone.get_log('stateEstimate.x')

        # si le drone cherche le bord sans le trouver au bout de 50cm,
        # c'est probablement que la plateforme était de l'autre côté
        if current_position < position_history - 0.5:
            drone.start_linear_motion(speed2_x, speed2_y, 0)
            position_history = -100
            print('trop loin')
            print(current_position, 'history', position_history)
        else:
            drone.start_linear_motion(-speed2_x, -speed2_y, 0)
            print('bien')
            print(current_position)
        
        edge_detected = edge_detection(drone, fly_height=height, threshold=0.03)
        
        if edge_detected:
            drone.stop()
            fly = False
        time.sleep(0.1)

    # stabilisation
    time.sleep(2)

    # on estime le centre de la box en fonction de là où il detecte un edge
    center_x = drone.get_log('stateEstimate.x') + half_plateform_x
    center_y = drone.get_log('stateEstimate.y') + half_plateform_y

    go_to_P(drone, center_x, center_y)

    drone.land()

    landed = 1

    return landed


def go_to_P(drone:Drone, x, y):
    """ Fonction go to avec contrôleur P """

    error = 1
    fly = True

    #screenshot de la pos au moment ou il detecte la boite
    position_estimate = [0,0]
    position_estimate[0] = drone.get_log('stateEstimate.x') 
    position_estimate[1] = drone.get_log('stateEstimate.y')


    print(f'Position drone x = {position_estimate[0]:.3f} y = {position_estimate[1]:.3}')
    print(f'Position box x = {x:.3f} y = {y:.3f}')
    print(f'Platform start x = {Platform.x_start:.3f} y = {Platform.y_start:.3f}')

    while(fly):
        drone.stop_by_hand()
        
        position_estimate[0] = drone.get_log('stateEstimate.x') 
        position_estimate[1] = drone.get_log('stateEstimate.y')

        # essaye de revenir au centre de la boite
        if error > 0.02 :
            landing_speed = -0.1
            P = 5.5
            err_x = (position_estimate[0]-x)
            err_y = (position_estimate[1]-y)
            landing_speed_x = err_x * landing_speed * P
            landing_speed_y = err_y * landing_speed * P

            drone.start_linear_motion(landing_speed_x,landing_speed_y, 0)
            error = np.sqrt(err_x**2+err_y**2)

        # si l erreur est suffisamment petite on atterit
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
        drone = Drone(scf, default_height=0.4)
        
        drone.start_logs()

        drone.take_off(0.2)

        landing_procedure(drone, direction=Direction.FORWARD, height=0.2, search_first_edge=True)
       
        drone.stop_logs()



        
       
        



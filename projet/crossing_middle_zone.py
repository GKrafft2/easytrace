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
from arena import Arena, Direction

def crossing_middle_zone(drone:Drone, central_line):

        arrival = zone_P2_detection(drone)
        speed_x, speed_y = obstacle_detection(drone, central_line, Direction.FORWARD)
            
        if not arrival: 
            drone.start_linear_motion(speed_x, speed_y, 0)
        else:
            drone.stop()

        return arrival

def obstacle_detection(drone:Drone, line_coord, direction:Direction):
    """ detect obstacle and return a pair of speed (x,y) to avoid
        Param: line_position the y line to follow from drone origin """

    # AVOID_DIST_LAT = 150  #mm # mise à jour plus loin
    AVOID_DIST_FRONT = 400  # mm
    AVOID_SPEED_LAT = 0.5 
    AVOID_SPEED_COME_BACK = 0.2
    AVOID_TIME_COME_BACK = 2 # secondes
    AVOID_SPEED_FRONT = 0.7
    FORWARD_SPEED = 0.2

    # cree un array pour mettre les 5 variables de sensor
    # range [front,back,left,right,up]
    range_sensors = np.empty(5)
    position_estimate = [0, 0]

    # Threshold pour le maintien de la ligne de suivi sans oscillations
    POSITION_DIRECTION_THRESH = 0.05

    correction = 0
    speed_east = 0
    speed_east_lat = 0
    speed_east_front = 0

    avoid_dir = drone.default_direction 

    # ajuste les capteurs en fonction de la direction de déplacement
    if direction == Direction.FORWARD: # référence
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

    # ======= adaptation de la distance latérale ==============
    # si le drone est sur la bonne trajectoire, il sera moins sensible que s'il revient en position
    if drone.on_track:
        AVOID_DIST_LAT = 50
    else:
        AVOID_DIST_LAT = 160


    # ====== évitement latéral =======
    if range_sensors[2] < AVOID_DIST_LAT:  # obstacle détecté à gauche
        drone.obstacle_lateral = True
        speed_east_lat = -AVOID_SPEED_LAT
    elif range_sensors[3] < AVOID_DIST_LAT:  # obstacle détecté à droite
        drone.obstacle_lateral = True
        speed_east_lat = AVOID_SPEED_LAT
    else:
        drone.obstacle_lateral = False
        speed_east_lat = 0


    # ====== évitement frontal ========
    if range_sensors[0] < AVOID_DIST_FRONT:
        drone.obstacle_frontal = True
        if position_estimate[1] > line_coord + 1:  # trop a gauche doit éviter par la droite
            avoid_dir = RIGHT
            drone.default_direction = RIGHT
        elif position_estimate[1] < line_coord - 1:  # trop a droite doit éviter par la gauche
            avoid_dir = LEFT
            drone.default_direction = LEFT

        speed_east_front = avoid_dir * AVOID_SPEED_FRONT
    else:
        speed_east_front = 0
        drone.obstacle_frontal = False


    # ====== update la vitesse en y ======
    speed_east = (speed_east_lat + speed_east_front)

    if speed_east != 0:
        correction = FORWARD_SPEED/2  # correction pour aller plus lentement quand il y a des obstacles
    
    # ====== revient à la ligne directrice si pas d'obstacle ===============
    if drone.obstacle_wait == False:
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

    # ===== Délai avant la déclaration de fin d'obstacle pour revenir sur la ligne de direction
    if drone.obstacle_frontal or drone.obstacle_lateral:
        drone.time1 = time.time_ns()
        drone.obstacle_wait = True
    if time.time_ns() - drone.time1 > AVOID_TIME_COME_BACK*1e9:
        drone.obstacle_wait = False
        
    speed_north = FORWARD_SPEED-correction

    # ajuste les vitesses du drone en x et y en fonction des vitesses calculés de l'orientation
    if direction == Direction.FORWARD:
        speed_x, speed_y = speed_north, speed_east
    elif direction == Direction.LEFT:
        speed_x, speed_y = -speed_east, speed_north
    elif direction == Direction.RIGHT:
        speed_x, speed_y = speed_east, -speed_north
    elif direction == Direction.BACKWARD:
        speed_x, speed_y = -speed_north, -speed_east
    
    return speed_x, speed_y


def zone_P2_detection(drone:Drone):
    """ vérifie quand le drone arrive dans la zone de la plateforme d'arrivée """

    arrival = False

    if drone.get_log('stateEstimate.x') >= Arena.START_ZONE_2:
        arrival = True

    return arrival


if __name__ == '__main__':
    # initalise les drivers low level, obligatoire
    cflib.crtp.init_drivers()
    # identifiant radio du drone
    URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E702')

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        # crée un drone (hérite de motion commander)
        drone = Drone(scf, default_height=0.2)

        #le drone suit la ligne au centre de l'arène
        central_line = - (Arena.ORIGIN_Y - Arena.WIDTH/2)
        # update de la direction par défault s'il y a un obstacle
        if central_line > 0:
            drone.default_direction = 1
        else:
            drone.default_direction = -1

        drone.start_logs()

        drone.take_off()

        crossed_middle_zone = False
        while(not crossed_middle_zone):
            drone.update_slam()
            drone.stop_by_hand()
            crossed_middle_zone = crossing_middle_zone(drone, central_line)
            time.sleep(0.1)

        drone.land()
        drone.slam.hold()

        drone.stop_logs(save=False)
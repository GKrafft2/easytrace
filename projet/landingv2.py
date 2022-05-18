# Libraries python
import time
import numpy as np
import matplotlib.pyplot as plt

# Libraries crazyflie
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper

# Libraries personnelles
from drone import Drone
from Arena import Arena
from p_crossing import avoid
from p_crossing import Direction



def edge_detection(drone:Drone):
    log = []
    position_estimate = [0, 0, 0, 0]
    edge_pos_1 = [0, 0]
    edge_pos_2 = [0, 0]
    init = False
    Fly = True
    first_pass = True
    second_pass = False
    THRESH = 0.010

    while(Fly):
        position_estimate[0] = drone.get_log('stateEstimate.x')
        position_estimate[1] = drone.get_log('stateEstimate.y')
        position_estimate[2] = drone.get_log('stateEstimate.z')
        position_estimate[3] = drone.get_log('range.zrange')/1000

        # Tableau "circulaire" des 5 derniers logs de estimate.z
        drone.zrange = np.append(drone.zrange, position_estimate[3])
        drone.zrange = drone.zrange[1:]
        log.append(position_estimate[0])
        print(drone.zrange)
        #print(np.mean(drone.zrange))

        if np.all(drone.zrange): init = True #si le tableau ne contient plus de 0

        if first_pass:
            drone.start_linear_motion(0.3, 0, 0)
        elif second_pass:
            drone.start_linear_motion(0, 0.3, 0)

        if init:
            moy = np.mean(drone.zrange[:-1])
            if drone.zrange[4] > moy + THRESH : #grosse variation = edge (en descente)

                if second_pass:
                    edge_pos_2[0] = position_estimate[0]  # screenshot de la position de l edge
                    edge_pos_2[1] = position_estimate[1]
                    print('second_edge detected')
                    print(edge_pos_1)
                    print(edge_pos_2)
                    drone.land()
                elif first_pass:
                    edge_pos_1[0] = position_estimate[0] # screenshot de la position de l edge
                    edge_pos_1[1] = position_estimate[1]
                    print('first edge detected')
                    first_pass = False
                    second_pass = True
                    init = False
                    drone.zrange = np.zeros(5)
                    time.sleep(0.5)


    # # Détecte un changement de hauteur selon le threshold = détection de la plateforme
    # THRESH = 0.03
    # moy = np.mean(drone.zrange[:-1])
    # print(moy-drone.zrange[-1])
    # if moy > drone.height_cmd-0.05 and (drone.zrange[-1] < moy - THRESH or drone.zrange[-1] > moy + THRESH): #detecte si on est passé au dessus de qqch (plateforme)
    #     #le mode landing est activé
    #     print("edge found") #passe la main au landing
    #     edge_detected = True
    #     drone.stop()
    #
    # # retourne la position position détectée
    # return edge_detected, position_estimate[0], position_estimate[1]
    #
    # if landing:
    #     print('landing')
    #     # screenshot de la pos au moment ou il detecte la boite
    #     if first_pass:
    #         limit_x = position_estimate[0]
    #         limit_y = position_estimate[1]
    #
    #         # on estime le centre de la box en fonction de la ou il detecte un edge et sa vitesse d'approche
    #         center_x = limit_x + np.sign(body_x_cmd) * demi_boite
    #         center_y = limit_y + np.sign(body_y_cmd) * demi_boite
    #
    #         print(f'pos boite = {center_x} y = {center_y}')
    #         first_pass = False
    #
    #     time.sleep(0.5)
    #
    #     print(f'error {error}')
    #
    #     # essaye de revenir au centre de la boite
    #     if error > 0.05:
    #         landing_speed = 0.1
    #         P = 5
    #         err_x = (position_estimate[0] - center_x)
    #         err_y = (position_estimate[1] - center_y)
    #         landing_speed_x = err_x * landing_speed * P
    #         landing_speed_y = err_y * landing_speed * P
    #
    #         # eviter que les vitesses deviennent trop grandes marche pas psk on risque d avoir des vitesses negatives
    #         # if landing_speed_x > 0.2:
    #         # landing_speed_x = 0.2
    #         # if landing_speed_y > 0.2:
    #         # landing_speed_y = 0.2
    #         print(landing_speed_x, landing_speed_y)
    #
    #         mc.start_linear_motion(landing_speed_x, landing_speed_y, 0)
    #         error = np.sqrt(err_x ** 2 + err_y ** 2)
    #     # si l erreur est suffisamment petite on atterit en breakant
    #     else:
    #         print('boite')
    #         fly = False
    #
    #     # mc.start_linear_motion(-body_x_cmd,-body_y_cmd, 0)#revient sur ses pas
    #     # time.sleep(1)#laisse le temps de revenir sur la plateforme
    #     # break#quitte le while donc atterit
    #
    # if not landing:
    #     mc.start_linear_motion(body_x_cmd, body_y_cmd, 0)
    #
    # time.sleep(0.1)

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

        edge_detection(drone)

        print("land")
        drone.land()
        drone.slam.slam_hold()

        drone.stop_logs(save=False)
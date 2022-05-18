
class Platform():
    """ Dimensions [m] de la plateforme de départ et arrivée """
    SIZE = 0.3 # même largeur et profondeur
    HEIGHT = 0.1
    HALF_BOX = 0.25
    # x_start = 0 # emplacement x trouvé de la boite 2
    # y_start = 0 # emplacement y trouvé de la boite 2

class Arena():
    """ Dimension en [m] l'arène """
    
    # Dimension absolue, origine bas droite
    WIDTH = 3
    LENGTH = 5

    # Landing and starting region
    REGION_LENGTH = 1.5

    # Position (centre) de la plateforme depuis l'origine
    ORIGIN_X = 0.3        # DEPUIS LE BAS !!!!
    ORIGIN_Y = 1.5      # DEPUIS LA DROITE !!!!

    # Distance de la zone d'arrivée
    START_ZONE_2 = LENGTH - REGION_LENGTH - ORIGIN_X

    # Limites de l'arène [m], à partir de la plateforme
    LIM_EAST = - ORIGIN_Y
    LIM_WEST = WIDTH - ORIGIN_Y
    LIM_SOUTH = - ORIGIN_X
    LIM_NORTH = LENGTH - ORIGIN_X
    LIM_UP = 2
    LIM_DOWN = 0

class Direction():
    FORWARD = 0
    RIGHT = 1
    LEFT = 2
    BACKWARD = 3


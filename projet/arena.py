
class platform():
    """ Dimensions [m] de la plateforme de départ et arrivée """
    SIZE = 0.3 # même largeur et profondeur
    HEIGHT = 0.1
    HALF_X = 0.22 # moitié de la boite, adaptée pour la réalité du drone
    HALF_Y = 0.12
    x_start = 0 # emplacement x trouvé de la boite 2
    y_start = 0 # emplacement y trouvé de la boite 2

class arena():
    """ Dimension en [m] l'arène """
    
    # Dimension absolue, origine bas droite
    WIDTH = 3
    HEIGHT = 5

    # Position de la plateforme depuis l'origine
    ORIGIN_X = 1        # DEPUIS LE BAS !!!!
    ORIGIN_Y = 0.3      # DEPUIS LA DROITE !!!!

    # Limites de l'arène [m], à partir de la plateforme
    LIM_EAST = - ORIGIN_Y
    LIM_WEST = WIDTH - ORIGIN_Y
    LIM_SOUTH = - ORIGIN_X
    LIM_NORTH = HEIGHT - ORIGIN_X
    LIM_UP = 2
    LIM_DOWN = 0




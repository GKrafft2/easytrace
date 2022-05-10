from enum import Enum

class arena_dim():
    """ Taille [m] absolue de l'arène """
    WIDTH = 3
    HEIGHT = 5

class arena_limits():
    """Limites de l'arène [m], à partir de la zone de départ (référentiel du drone)"""
    EAST = 1
    WEST = 2
    SOUTH = 0.5
    NORTH = 4.5

class platform():
    """ Dimensions [m] de la plateforme de départ et arrivée """
    SIZE = 0.3 # même largeur et profondeur
    HEIGHT = 0.1
    HALF_X = 0.22 # moitié de la boite, adaptée pour la réalité du drone
    HALF_Y = 0.12
    x_start = 0 # emplacement x trouvé de la boite 2
    y_start = 0 # emplacement y trouvé de la boite 2
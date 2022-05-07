from enum import Enum

class arena_dim(Enum):
    """ Taille [m] absolue de l'arène """
    WIDTH = 3
    HEIGHT = 5

class arena_limits(Enum):
    """Limites de l'arène [m], à partir de la zone de départ (référentiel du drone)"""
    EAST = 1
    WEST = 2
    SOUTH = 0.5
    NORTH = 4.5

class platform(Enum):
    """ Dimensions [m] de la plateforme de départ et arrivée """
    SIZE = 0.3 # même largeur et profondeur
    HEIGHT = 0.1
    HALF = 0.1 # moitié de la boite, adaptée pour la réalité du drone
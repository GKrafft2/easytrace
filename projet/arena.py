from enum import Enum

class arena_dim(Enum):
    """ Taille [mm] absolue de l'arène """
    WIDTH = 3000
    HEIGHT = 5000

class arena_limits(Enum):
    """Limites de l'arène [mm], à partir de la zone de départ (référentiel du drone)"""
    EAST = 1000
    WEST = 2000
    SOUTH = 500
    NORTH = 4500

class platform(Enum):
    """ Dimensions [mm] de la plateforme de départ et arrivée """
    BOX_SIZE = 300.0 # même largeur et profondeur
    BOX_HEIGHT = 100.0
    BOX_HALF = 100.0 # moitié de la boite, adaptée pour la réalité du drone
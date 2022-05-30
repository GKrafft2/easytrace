
class Platform():
    """ Start and goal platform dimensions [m] """
    SIZE = 0.3 # width
    HEIGHT = 0.1
    HALF_BOX = 0.17

class Arena():
    """ Arena dimension [m] """
    
    # absolute dimention, origin down right
    WIDTH = 3.1
    LENGTH = 5

    # Landing and starting region
    REGION_LENGTH = 1.5

    # Position (centre) of the plateforme from the origine
    ORIGIN_X = 1      # from the "bottum"
    ORIGIN_Y = 2     # from the right

    # Distance of the drone to the landing region
    START_ZONE_2 = LENGTH - REGION_LENGTH - ORIGIN_X

    # Arena limits [m], from the start platform
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


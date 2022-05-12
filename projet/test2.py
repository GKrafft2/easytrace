from matplotlib.animation import FuncAnimation
import matplotlib.pyplot as plt

import numpy as np

import cv2

def test1():
    map = np.zeros((500, 300))

    i = 0

    # ideas :
    # use index for path (128) obstacles (255)
    # expand on in and use more complex indexing like 200 for obstacles who are probably here
    # use increments and thresholding for obstacles but index (64) for path

    for i in range(500):
        map[i][150] = 64 # path
        map[i][100] = 255 # obstacle
        map[i][200] = 255 # obstacle
        # Makes it into grayscale
        graymap = map.astype('uint8')
        # change colormap for more pzazz (si les valeurs sont au dessus de 255 ca reboucle)
        colormap = cv2.applyColorMap(graymap,  cv2.COLORMAP_JET)
        # Resize for better viewing (dimensions seems to be inverted)
        imS = cv2.resize(colormap, (map.shape[1]*2, map.shape[0]))
        cv2.imshow('image', imS)
        cv2.waitKey(1)
        print(i)
    cv2.waitKey(0) # the image stays displayed as long as there is no imput but the program also stops
    # save image here
    print("test")

logs = np.array([[1, 2, 3, 4, 5, 6, 7, 0, 0 ,0 ,0, 0, 0, 0, 0 ]])
logs = np.transpose(logs)
count = 7

def test2(variable, index=None):
    """ 
    Si index est donné, retourne les x derniers logs
    Autrement retourne le dernier logs enregistré de l'index de variable donné """

    # retourne l'index de la variable de log
    idx_variable = variable

    if index is None:
        return logs[-1][idx_variable]
    else:
        return logs[count-index:count:,idx_variable]


if __name__ == '__main__':

    log = test2(0, 6)
    print(log[:-1])

    print(log)





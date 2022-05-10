import matplotlib.pyplot as plt
import numpy as np

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time

M = np.zeros((100,100))

def animate(i):
    dataArray = np.random.rand(100,100)
    matrice.set_array(dataArray)

fig,ax = plt.subplots()
matrice = ax.matshow(M)
plt.colorbar(matrice)

ani = animation.FuncAnimation(fig, animate, interval=30)
plt.show()
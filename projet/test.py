import matplotlib.pyplot as plt
import numpy as np
from time import time

map = np.zeros((1000,1000))

fig = plt.imshow(map)

fig.show()
fig.canvas.draw()

plt.ion()

for i in range(1000):
    t1 = time()
    map[i][i] = 1
    if i%20 == 0:
        #plt.imshow(map)
        fig.canvas.draw()
        fig.canvas.flush_events()
    print(f'frame time  = {time()-t1}')


# %% 
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
from scipy.signal import correlate

# import data in pandas dataframe (change the file name to your local file)
data = pd.read_csv('logs/2022_05_07_20_54_43.csv', header = None)
print(data)

# remove all-zeros rows
data = data[(data.T != 0).any()]
print(data)

data_numpy = np.array(data[2])-0.4
len_data = int(len(data_numpy)/2)


matchx = np.arange(0,1100-800, dtype=float)-0.4
# match = np.sin(2*np.pi/150*matchx+np.pi)*0.05+0.01
# match = ((matchx - 50)/100)**2/5

match = data_numpy[800:1100]

output = np.correlate(data_numpy[:len_data], match, "same")
time = np.arange(0, output.shape[0])

e1 = np.argmax(output)
output = np.correlate(data_numpy[len_data:], match, "same")
e2 = np.argmax(output) + len_data
# plot all timeseries
plt.figure()
for i in data:
    data[i].plot(label=f'{i}')
    plt.plot(matchx, match)
    # plt.plot(time, output)
    plt.axvline(e1)
    plt.axvline(e2)
plt.legend()

# # plot only one timeserie
# plt.figure()
# data[0].plot()
# data[1].plot()
# data[2].plot()


plt.legend()


plt.show()

# %%

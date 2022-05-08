import matplotlib.pyplot as plt
import pandas as pd

# import data in pandas dataframe (change the file name to your local file)
data = pd.read_csv('../projet/logs/2022_05_08_14_52_23.csv', header = None)
print(data)

# remove all-zeros rows
data = data[(data.T != 0).any()]
print(data)

# plot all timeseries
plt.figure()
for i in data:
    data[i].plot(label=f'{i}')
plt.legend()
plt.show()

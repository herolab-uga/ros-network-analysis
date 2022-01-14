import csv
from matplotlib import pyplot as plt
import pandas as pd

col_list = ["network_delay"]

df = pd.read_csv("combined_sync_data.csv", usecols=col_list, index_col=False)

df = df.loc[df['network_delay'] != -1]

network_delay = df["network_delay"]
network_delay= network_delay[(network_delay.notnull()) & (network_delay >0)]

fig = plt.figure(dpi = 128, figsize = (10,6))
plt.plot(network_delay, c = 'red') #Line 1

plt.xlabel('Time Step',fontsize = 20)
plt.ylabel("Delay (ms)", fontsize = 20)
plt.tick_params(axis = 'both', which = 'major' , labelsize = 8)
fig1 = plt.gcf()
plt.draw()
fig1.savefig('delay.png',dpi=128)
plt.show()

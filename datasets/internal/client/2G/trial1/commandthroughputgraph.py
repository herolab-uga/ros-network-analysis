import csv
from matplotlib import pyplot as plt
import pandas as pd

col_list = ["network_delay", "s_total_mbps", "s_total_tx_mbps","s_total_rx_mbps"]

df = pd.read_csv("combined_sync_data.csv", usecols=col_list, index_col=False)


df = df.loc[df['network_delay'] != -1]

c_total_mbps = df["s_total_mbps"]
c_total_mbps= c_total_mbps[(c_total_mbps.notnull()) & (c_total_mbps < 200) & (c_total_mbps >=0)]

c_total_tx_mbps = df["s_total_tx_mbps"]
c_total_tx_mbps= c_total_tx_mbps[(c_total_tx_mbps.notnull()) & (c_total_tx_mbps < 200) & (c_total_tx_mbps >=0)]

c_total_rx_mbps = df["s_total_rx_mbps"]
c_total_rx_mbps= c_total_rx_mbps[(c_total_rx_mbps.notnull()) & (c_total_rx_mbps < 200) & (c_total_rx_mbps >=0)]



fig = plt.figure(dpi = 128, figsize = (10,6))
plt.plot(c_total_mbps, c = 'red',marker = "*") #Line 1
plt.plot(c_total_tx_mbps, c = 'green', marker = ".", linestyle = '--') #Line 2
plt.plot(c_total_rx_mbps, c = 'blue', marker = "x", linestyle = '--') #Line 3
#Format Plot
plt.xlabel('Time Step',fontsize = 20)
plt.ylabel("Throughput (mbps)", fontsize = 20)
plt.legend(['total throughput', 'tx throughput', 'rx throughput'])
#plt.tick_params(axis = 'both', which = 'major' , labelsize = 8)
fig1 = plt.gcf()
plt.draw()
fig1.savefig('commandthroughput.png',dpi=128)
plt.show()


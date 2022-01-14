import csv
from matplotlib import pyplot as plt
import pandas as pd

col_list = ["c_rssi", "c_rssi2", "s_rssi", "s_rssi2"]

df = pd.read_csv("combined_sync_data.csv", usecols=col_list, index_col=False)
#print(df["network_delay"].count())
#df = df.loc[df['network_delay'] != -1]

c_rssi = df["c_rssi"]
c_rssi= c_rssi[(c_rssi.notnull()) & (c_rssi < -4) & (c_rssi > -200)]


c_rssi2 = df["c_rssi2"]
c_rssi2= c_rssi2[(c_rssi2.notnull()) & (c_rssi2 > -200) & (c_rssi2 < -4)]


s_rssi = df["s_rssi"]
s_rssi= s_rssi[(s_rssi.notnull()) & (s_rssi > -200) & (s_rssi < -4)]


s_rssi2 = df["s_rssi2"]
s_rssi2= s_rssi2[(s_rssi2.notnull()) & (s_rssi2 > -200) & (s_rssi2 < -4)]


fig = plt.figure(dpi = 128, figsize = (10,6))
#plt.plot(c_rssi, c = 'orange',marker = "*") #Line 1
plt.plot(c_rssi2, c = 'salmon', marker = ".") #Line 2
#plt.plot(s_rssi, c = 'lime', marker = "x", linestyle = '--') #Line 3
plt.plot(s_rssi2, c = 'cyan', marker = "+", linestyle = '--') #Line 4
#Format Plot
plt.xlabel('Time Step',fontsize = 20)
plt.ylabel("RSSI (dBm)", fontsize = 20)
plt.legend(['Robot iface 3E or 7','station iface 3E or 7'])
plt.tick_params(axis = 'both', which = 'major' , labelsize = 8)
fig1 = plt.gcf()
plt.draw()
fig1.savefig('rssi.png',dpi=128)
plt.show()


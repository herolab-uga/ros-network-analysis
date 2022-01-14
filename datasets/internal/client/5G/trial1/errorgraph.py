import csv
from matplotlib import pyplot as plt
import pandas as pd
col_list = ["c_retransmits", "c_tx_retires","s_retransmits", "s_tx_retires","s_retransmits2", "s_tx_retires2"]

df = pd.read_csv("combined_sync_data.csv", usecols=col_list, index_col=False)

c_retransmits = df["c_retransmits"]
c_retransmits = c_retransmits[(c_retransmits.notnull()) & (c_retransmits >=0)]
c_retransmits_new = c_retransmits.diff()
c_retransmits_new = c_retransmits_new[(c_retransmits_new.notnull()) & (c_retransmits_new >=0)]
	

s_retransmits = df["s_retransmits"]
s_retransmits= s_retransmits[(s_retransmits.notnull()) & (s_retransmits >=0)]
s_retransmits_new = s_retransmits.diff()
s_retransmits_new = s_retransmits_new[(s_retransmits_new.notnull()) & (s_retransmits_new >=0)]


s_retransmits2 = df["s_retransmits2"]
s_retransmits2= s_retransmits2[(s_retransmits2.notnull()) & (s_retransmits2 >=0)]
s_retransmits2_new = s_retransmits2.diff()
s_retransmits2_new = s_retransmits2_new[(s_retransmits2_new.notnull()) & (s_retransmits2_new >=0)]

	


fig = plt.figure(dpi = 128, figsize = (10,6))
plt.plot(c_retransmits_new, c = 'red',marker = "*") #Line 1
plt.plot(s_retransmits_new, c = 'green', marker = ".", linestyle = '--') #Line 2
#plt.plot(s_retransmits2_new, c = 'blue', marker = "x", linestyle = '--') #Line 3
#Format Plot
plt.xlabel('Time Step',fontsize = 20)
plt.ylabel("Retransmits", fontsize = 20)
plt.legend(['robot iface 1*', 'station iface 3*'])
#plt.tick_params(axis = 'both', which = 'major' , labelsize = 8)
fig1 = plt.gcf()
plt.draw()
fig1.savefig('error.png',dpi=128)
plt.show()


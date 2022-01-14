import csv
import pandas as c_total_tx_mbps

col_list = ["c_rssi", "c_lqi", "c_rssi_status", "c_rssi2", "c_lqi2", "c_rssi_status2", "s_rssi", "s_lqi", "s_rssi_status", "s_rssi2", "s_lqi2", "s_rssi_status2"]

df = c_total_tx_mbps.read_csv("combined_sync_data.csv", usecols=col_list, index_col=False)
#print(df["network_delay"].count())
#df = df.loc[df['network_delay'] != -1]

c_rssi = df["c_rssi"]
c_rssi= c_rssi[(c_rssi.notnull()) & (c_rssi < -4) & (c_rssi > -200)]

c_lqi = df["c_lqi"]
c_lqi= c_lqi[(c_lqi.notnull()) & (c_lqi >= 1) & (c_lqi <= 100)]



c_rssi2 = df["c_rssi2"]
c_rssi2= c_rssi2[(c_rssi2.notnull()) & (c_rssi2 > -200) & (c_rssi2 < -4)]

c_lqi2 = df["c_lqi2"]
c_lqi2= c_lqi2[(c_lqi2.notnull()) & (c_lqi2 >= 1) & (c_lqi2 <= 100)]



s_rssi = df["s_rssi"]
s_rssi= s_rssi[(s_rssi.notnull()) & (s_rssi > -200) & (s_rssi < -4)]

s_lqi = df["s_lqi"]
s_lqi= s_lqi[(s_lqi.notnull()) & (s_lqi >= 1) & (s_lqi <= 100)]



s_rssi2 = df["s_rssi2"]
s_rssi2= s_rssi2[(s_rssi2.notnull()) & (s_rssi2 > -200) & (s_rssi2 < -4)]

s_lqi2 = df["s_lqi2"]
s_lqi2= s_lqi2[(s_lqi2.notnull()) & (s_lqi2 >= 1) & (s_lqi2 <= 100)]



print(c_rssi)

avg_c_rssi=c_rssi.mean()
print(avg_c_rssi)

avg_c_lqi=c_lqi.mean()
print(avg_c_lqi)



avg_c_rssi2=c_rssi2.mean()
print(avg_c_rssi2)

avg_c_lqi2=c_lqi2.mean()
print(avg_c_lqi2)


avg_s_rssi=s_rssi.mean()
print(avg_s_rssi)

avg_s_lqi=s_lqi.mean()
print(avg_s_lqi)



avg_s_rssi2=s_rssi2.mean()
print(avg_s_rssi2)

avg_s_lqi2=s_lqi2.mean()
print(avg_s_lqi2)


std_c_rssi=c_rssi.std()
print(std_c_rssi)

std_c_lqi=c_lqi.std()
print(std_c_lqi)


std_c_rssi2=c_rssi2.std()
print(std_c_rssi2)

std_c_lqi2=c_lqi2.std()
print(std_c_lqi2)


std_s_rssi=s_rssi.std()
print(std_s_rssi)

std_s_lqi=s_lqi.std()
print(std_s_lqi)


std_s_rssi2=s_rssi2.std()
print(std_s_rssi2)

std_s_lqi2=s_lqi2.std()
print(std_s_lqi2)


rows=[['avg_c_rssi',avg_c_rssi],
	['avg_c_lqi',avg_c_lqi],
	['avg_c_rssi2',avg_c_rssi2],
	['avg_c_lqi2',avg_c_lqi2],
	['avg_s_rssi',avg_s_rssi],
	['avg_s_lqi',avg_s_lqi],
	['avg_s_rssi2',avg_s_rssi2],
	['avg_s_lqi2',avg_s_lqi2],
	['std_c_rssi',std_c_rssi],
	['std_c_lqi',std_c_lqi],
	['std_c_rssi2',std_c_rssi2],
	['std_c_lqi2',std_c_lqi2],
	['std_s_rssi',std_s_rssi],
	['std_s_lqi',std_s_lqi],
	['std_s_rssi2',std_s_rssi2],
	['std_s_lqi2',std_s_lqi2]]


filename="rssi.csv"
with open(filename, 'w') as csvfile:
	csvwriter = csv.writer(csvfile)
    	csvwriter.writerows(rows)

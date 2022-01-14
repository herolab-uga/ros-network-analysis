import csv
import pandas as pd

col_list = ["network_delay", "c_total_tx_mbps", "c_total_rx_mbps", "c_total_mbps","c_total_tx_mbps2", "c_total_rx_mbps2", "c_total_mbps2","s_total_tx_mbps","s_total_rx_mbps","s_total_mbps", "s_total_tx_mbps2", "s_total_rx_mbps2", "s_total_mbps2" ]

df = pd.read_csv("combined_sync_data.csv", usecols=col_list, index_col=False)


df = df.loc[df['network_delay'] != -1]

c_total_tx_mbps = df["c_total_tx_mbps"]
c_total_tx_mbps= c_total_tx_mbps[(c_total_tx_mbps.notnull()) & (c_total_tx_mbps < 200) & (c_total_tx_mbps >=0)]

c_total_rx_mbps = df["c_total_rx_mbps"]
c_total_rx_mbps= c_total_rx_mbps[(c_total_rx_mbps.notnull()) & (c_total_rx_mbps < 200) & (c_total_rx_mbps >=0)]

c_total_mbps = df["c_total_mbps"]
c_total_mbps= c_total_mbps[(c_total_mbps.notnull()) & (c_total_mbps < 200) & (c_total_mbps >=0)]

c_total_tx_mbps2 = df["c_total_tx_mbps2"]
c_total_tx_mbps2= c_total_tx_mbps2[(c_total_tx_mbps2.notnull()) & (c_total_tx_mbps2 < 200) & (c_total_tx_mbps2 >=0)]

c_total_rx_mbps2 = df["c_total_rx_mbps2"]
c_total_rx_mbps2= c_total_rx_mbps2[(c_total_rx_mbps2.notnull()) & (c_total_rx_mbps2 < 200) & (c_total_rx_mbps2 >=0)]

c_total_mbps2 = df["c_total_mbps2"]
c_total_mbps2= c_total_mbps2[(c_total_mbps2.notnull()) & (c_total_mbps2 < 200) & (c_total_mbps2 >=0)]

s_total_tx_mbps = df["s_total_tx_mbps"]
s_total_tx_mbps= s_total_tx_mbps[(s_total_tx_mbps.notnull()) & (s_total_tx_mbps < 200) & (s_total_tx_mbps >=0)]

s_total_rx_mbps = df["s_total_rx_mbps"]
s_total_rx_mbps= s_total_rx_mbps[(s_total_rx_mbps.notnull()) & (s_total_rx_mbps < 200) & (s_total_rx_mbps >=0)]

s_total_mbps = df["s_total_mbps"]
s_total_mbps= s_total_mbps[(s_total_mbps.notnull()) & (s_total_mbps < 200) & (s_total_mbps >=0)]

s_total_tx_mbps2 = df["s_total_tx_mbps2"]
s_total_tx_mbps2= s_total_tx_mbps2[(s_total_tx_mbps2.notnull()) & (s_total_tx_mbps2 < 200) & (s_total_tx_mbps2 >=0)]

s_total_rx_mbps2 = df["s_total_rx_mbps2"]
s_total_rx_mbps2= s_total_rx_mbps2[(s_total_rx_mbps2.notnull()) & (s_total_rx_mbps2 < 200) & (s_total_rx_mbps2 >=0)]

s_total_mbps2 = df["s_total_mbps2"]
s_total_mbps2= s_total_mbps2[(s_total_mbps2.notnull()) & (s_total_mbps2 < 200) & (s_total_mbps2 >=0)]


print(c_total_tx_mbps)

avg_c_total_tx_mbps=c_total_tx_mbps.mean()
print(avg_c_total_tx_mbps)

avg_c_total_rx_mbps=c_total_rx_mbps.mean()
print(avg_c_total_rx_mbps)

avg_c_total_mbps=c_total_mbps.mean()
print(avg_c_total_mbps)

avg_c_total_tx_mbps2=c_total_tx_mbps2.mean()
print(avg_c_total_tx_mbps2)

avg_c_total_rx_mbps2=c_total_rx_mbps2.mean()
print(avg_c_total_rx_mbps2)

avg_c_total_mbps2=c_total_mbps2.mean()
print(avg_c_total_mbps2)

avg_s_total_tx_mbps=s_total_tx_mbps.mean()
print(avg_s_total_tx_mbps)

avg_s_total_rx_mbps=s_total_rx_mbps.mean()
print(avg_s_total_rx_mbps)

avg_s_total_mbps=s_total_mbps.mean()
print(avg_s_total_mbps)

avg_s_total_tx_mbps2=s_total_tx_mbps2.mean()
print(avg_s_total_tx_mbps2)

avg_s_total_rx_mbps2=s_total_rx_mbps2.mean()
print(avg_s_total_rx_mbps2)

avg_s_total_mbps2=s_total_mbps2.mean()
print(avg_s_total_mbps2)

std_c_total_tx_mbps=c_total_tx_mbps.std()
print(std_c_total_tx_mbps)

std_c_total_rx_mbps=c_total_rx_mbps.std()
print(std_c_total_rx_mbps)

std_c_total_mbps=c_total_mbps.std()
print(std_c_total_mbps)

std_c_total_tx_mbps2=c_total_tx_mbps2.std()
print(std_c_total_tx_mbps2)

std_c_total_rx_mbps2=c_total_rx_mbps2.std()
print(std_c_total_rx_mbps2)

std_c_total_mbps2=c_total_mbps2.std()
print(std_c_total_mbps2)

std_s_total_tx_mbps=s_total_tx_mbps.std()
print(std_s_total_tx_mbps)

std_s_total_rx_mbps=s_total_rx_mbps.std()
print(std_s_total_rx_mbps)

std_s_total_mbps=s_total_mbps.std()
print(std_s_total_mbps)

std_s_total_tx_mbps2=s_total_tx_mbps2.std()
print(std_s_total_tx_mbps2)

std_s_total_rx_mbps2=s_total_rx_mbps2.std()
print(std_s_total_rx_mbps2)

std_s_total_mbps2=s_total_mbps2.std()
print(std_s_total_mbps2)


rows=[['avg_c_total_tx_mbps',avg_c_total_tx_mbps],
	['avg_c_total_rx_mbps',avg_c_total_rx_mbps],
	['avg_c_total_mbps',avg_c_total_mbps],
	['avg_c_total_tx_mbps2',avg_c_total_tx_mbps2],
	['avg_c_total_rx_mbps2',avg_c_total_rx_mbps2],
	['avg_c_total_mbps2',avg_c_total_mbps2],
	['avg_s_total_tx_mbps',avg_s_total_tx_mbps],
	['avg_s_total_rx_mbps',avg_s_total_rx_mbps],
	['avg_s_total_mbps',avg_s_total_mbps],
	['avg_s_total_tx_mbps2',avg_s_total_tx_mbps2],
	['avg_s_total_rx_mbps2',avg_s_total_rx_mbps2],
	['avg_s_total_mbps2',avg_s_total_mbps2],
	['std_c_total_tx_mbps',std_c_total_tx_mbps],
	['std_c_total_rx_mbps',std_c_total_rx_mbps],
	['std_c_total_mbps',std_c_total_mbps],
	['std_c_total_tx_mbps2',std_c_total_tx_mbps2],
	['std_c_total_rx_mbps2',std_c_total_rx_mbps2],
	['std_c_total_mbps2',std_c_total_mbps2],
	['std_s_total_tx_mbps',std_s_total_tx_mbps],
	['std_s_total_rx_mbps',std_s_total_rx_mbps],
	['std_s_total_mbps',std_s_total_mbps],
	['std_s_total_tx_mbps2',std_s_total_tx_mbps2],
	['std_s_total_rx_mbps2',std_s_total_rx_mbps2],
	['std_s_total_mbps2',std_s_total_mbps2]]


filename="throughput.csv"
with open(filename, 'w') as csvfile:
	csvwriter = csv.writer(csvfile)
    	csvwriter.writerows(rows)

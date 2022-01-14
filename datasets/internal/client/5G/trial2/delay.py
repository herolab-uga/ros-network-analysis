import csv
import pandas as c_total_tx_mbps

col_list = ["network_delay"]

df = c_total_tx_mbps.read_csv("combined_sync_data.csv", usecols=col_list, index_col=False)

#='file:///home/herolab/oneinface/direct/client/2G/trial1/throughput.csv'#$throughput.B9
df = df.loc[df['network_delay'] != -1]

network_delay = df["network_delay"]
network_delay= network_delay[(network_delay.notnull()) & (network_delay >0)]



print(network_delay)

avg_network_delay=network_delay.mean()
print(avg_network_delay)


std_network_delay=network_delay.std()
print(std_network_delay)

rows=[['avg_network_delay',avg_network_delay],
	['std_network_delay',std_network_delay]]
filename="delay.csv"
with open(filename, 'w') as csvfile:
	csvwriter = csv.writer(csvfile)
    	csvwriter.writerows(rows)

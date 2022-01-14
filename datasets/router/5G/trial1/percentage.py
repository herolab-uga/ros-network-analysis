import csv
import pandas as pd

col_list = ["network_delay"]

df = pd.read_csv("combined_sync_data.csv", usecols=col_list, index_col=False)

#='file:///home/herolab/oneinface/direct/client/2G/trial1/throughput.csv'#$throughput.B9
df = df.loc[df['network_delay'] != -2]
network_delay = df.loc[df['network_delay'] == -1]

network_delay_prc = network_delay.count()/df.count()
network_delay_prc = network_delay_prc*100

print(df.count())
print(network_delay.count())
print(network_delay_prc)



rows=[['percentage_delay',network_delay_prc]]
filename="percentage.csv"
with open(filename, 'w') as csvfile:
	csvwriter = csv.writer(csvfile)
    	csvwriter.writerows(rows)

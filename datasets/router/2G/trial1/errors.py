import csv
import pandas as pd
import numpy

col_list = ["c_retransmits", "c_tx_retires","s_retransmits", "s_tx_retires","s_retransmits2", "s_tx_retires2"]

df = pd.read_csv("combined_sync_data.csv", usecols=col_list, index_col=False)

c_retransmits = df["c_retransmits"]
c_retransmits = c_retransmits[(c_retransmits.notnull()) & (c_retransmits >=0)]
c_retransmits_new = c_retransmits.diff()
c_retransmits_new = c_retransmits_new[(c_retransmits_new.notnull()) & (c_retransmits_new >=0)]
	

c_tx_retires = df["c_tx_retires"]
c_tx_retires= c_tx_retires[(c_tx_retires.notnull()) & (c_tx_retires >=0)]
c_tx_retires_new = c_tx_retires.diff()
c_tx_retires_new = c_tx_retires_new[(c_tx_retires_new.notnull()) & (c_tx_retires_new >=0)]


s_retransmits = df["s_retransmits"]
s_retransmits= s_retransmits[(s_retransmits.notnull()) & (s_retransmits >=0)]
s_retransmits_new = s_retransmits.diff()
s_retransmits_new = s_retransmits_new[(s_retransmits_new.notnull()) & (s_retransmits_new >=0)]


s_tx_retires = df["s_tx_retires"]
s_tx_retires= s_tx_retires[(s_tx_retires.notnull()) & (s_tx_retires >=0)]
s_tx_retires_new = s_tx_retires.diff()
s_tx_retires_new = s_tx_retires_new[(s_tx_retires_new.notnull()) & (s_tx_retires_new >=0)]


s_retransmits2 = df["s_retransmits2"]
s_retransmits2= s_retransmits2[(s_retransmits2.notnull()) & (s_retransmits2 >=0)]
s_retransmits2_new = s_retransmits2.diff()
s_retransmits2_new = s_retransmits2_new[(s_retransmits2_new.notnull()) & (s_retransmits2_new >=0)]

	
s_tx_retires2 = df["s_tx_retires2"]
s_tx_retires2= s_tx_retires2[(s_tx_retires2.notnull()) & (s_tx_retires2 >=0)]
s_tx_retires2_new = s_tx_retires2.diff()
s_tx_retires2_new = s_tx_retires2_new[(s_tx_retires2_new.notnull()) & (s_tx_retires2_new >=0)]



avg_c_retransmits=c_retransmits_new.mean()
print(avg_c_retransmits)

avg_c_tx_retires=c_tx_retires_new.mean()
print(avg_c_tx_retires)

avg_s_retransmits=s_retransmits_new.mean()
print(avg_s_retransmits)

avg_s_tx_retires=s_tx_retires_new.mean()
print(avg_s_tx_retires)

avg_s_retransmits2=s_retransmits2_new.mean()
print(avg_s_retransmits2)

avg_s_tx_retires2=s_tx_retires2_new.mean()
print(avg_s_tx_retires2)

sum_c_retransmits=c_retransmits_new.sum()
print(sum_c_retransmits)

sum_c_tx_retires=c_tx_retires_new.sum()
print(sum_c_tx_retires)

sum_s_retransmits=s_retransmits_new.sum()
print(sum_s_retransmits)

sum_s_tx_retires=s_tx_retires_new.sum()
print(sum_s_tx_retires)

sum_s_retransmits2=s_retransmits2_new.sum()
print(sum_s_retransmits2)

sum_s_tx_retires2=s_tx_retires2_new.sum()
print(sum_s_tx_retires2)

std_c_retransmits=c_retransmits_new.std()
print(std_c_retransmits)

std_c_tx_retires=c_tx_retires_new.std()
print(std_c_tx_retires)

std_s_retransmits=s_retransmits_new.std()
print(std_s_retransmits)

std_s_tx_retires=s_tx_retires_new.std()
print(std_s_tx_retires)

std_s_retransmits2=s_retransmits2_new.std()
print(std_s_retransmits2)

std_s_tx_retires2=s_tx_retires2_new.std()
print(std_s_tx_retires2)



rows=[['avg_c_retransmits',avg_c_retransmits],
	['avg_c_tx_retires',avg_c_tx_retires],
	['avg_s_retransmits',avg_s_retransmits],
	['avg_s_tx_retires',avg_s_tx_retires],
	['avg_s_retransmits2',avg_s_retransmits2],
	['avg_s_tx_retires2',avg_s_tx_retires2],
	['sum_c_retransmits',sum_c_retransmits],
	['sum_c_tx_retires',sum_c_tx_retires],
	['sum_s_retransmits',sum_s_retransmits],
	['sum_s_tx_retires',sum_s_tx_retires],
	['sum_s_retransmits2',sum_s_retransmits2],
	['sum_s_tx_retires2',sum_s_tx_retires2],
	['std_c_retransmits',std_c_retransmits],
	['std_c_tx_retires',std_c_tx_retires],
	['std_s_retransmits',std_s_retransmits],
	['std_s_tx_retires',std_s_tx_retires],
	['std_s_retransmits2',std_s_retransmits2],
	['std_s_tx_retires2',std_s_tx_retires2],]
filename="errors.csv"
with open(filename, 'w') as csvfile:
	csvwriter = csv.writer(csvfile)
    	csvwriter.writerows(rows)

import csv
from matplotlib import pyplot as plt
import pandas as pd

col_list = ["robot_pos_x", "robot_pos_y"]

df = pd.read_csv("combined_sync_data.csv", usecols=col_list, index_col=False)


robot_pos_x = df["robot_pos_x"]
robot_pos_y = df["robot_pos_y"]


fig = plt.figure(dpi = 128, figsize = (10,6))
plt.plot(robot_pos_x,robot_pos_y, c = 'red',) #Line 1

plt.xlabel('x coordinate (m)',fontsize = 20)
plt.ylabel("y coordinate (m)", fontsize = 20)
#plt.legend(['total throughput', 'tx throughput', 'rx throughput'])
#plt.tick_params(axis = 'both', which = 'major' , labelsize = 8)
fig1 = plt.gcf()
plt.draw()
fig1.savefig('odom.png',dpi=128)
plt.show()


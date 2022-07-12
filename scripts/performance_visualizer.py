import csv
import plistlib
import matplotlib.pyplot as plt

dt = []
ratio = []
with open('../agent_performance_r20_sc.csv', newline='') as csvfile:
    spamreader = csv.reader(csvfile, delimiter=',')
    for row in spamreader:
        dt.append(float(row[2]))
        ratio.append(float(row[3]))
        
dt=dt[20:]
ratio=ratio[20:]
plt.plot(dt, ratio)
plt.show()
import csv
import plistlib
import matplotlib.pyplot as plt

dt = []
ratio = []
with open('../agent_performance_r7_sc_nc.csv', newline='') as csvfile:
    spamreader = csv.reader(csvfile, delimiter=',')
    for row in spamreader:
        dt.append(float(row[2]))
        ratio.append(float(row[3]))


# count result
mean_ratio = sum(ratio)/len(ratio)
mean_fps = 20*mean_ratio
print("mean_ratio:{:.2f}, mean_fps:{:.2f}".format(mean_ratio, mean_fps))
        
dt=dt[20:]
ratio=ratio[20:]
plt.plot(dt, ratio)
plt.show()
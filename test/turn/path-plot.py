#!/usr/bin/env python3

#
# Generates a 2D plot from the turn data
#

import seaborn as sns
import pandas as pd
import matplotlib
matplotlib.rcParams['text.usetex'] = True
from matplotlib import pyplot as plt
import math
import matplotlib.lines as mlines

# reads the data and calculates the median path
data = pd.read_csv('turns-data.csv')

sns.set_style("whitegrid")

fig, ax1 = plt.subplots()

#my_pal = {"t1": "#2a80b9", "t2": "#50af61", "omega1":"#904fad", "omega2":"#e77e27", "pi3":"#c1392b", "pi4":"#95a5a5"}
my_pal = {"t1": "#5DA5DA", "t2": "#60BD68", "omega1":"#FCD838", "omega2":"#FAA43A", 
    "pi3":"#F15854", "pi4":"#B276B2", "pi5":"#96613D", "pi6":"#979797"}

turnTypes = ['t1', 't2', 'omega1', 'omega2', 'pi3', 'pi4', 'pi5', 'pi6']
turnTypes.reverse()
for turnType in turnTypes:
    turnData = data[data['type'] == turnType]
    iteration = turnData.sort_values('duration').iloc[math.floor(len(turnData) / 2)]['iteration']

    name = turnType + '-' + str(iteration)
    tum = open('tum/' + name + '-full-path.tum', 'r')
    tumMedian = open('tum/' + turnType + '-median-path.tum', 'w')
    for line in tum:
        lineSplit = line.split()
        timestamp = float(lineSplit[0])
        x = float(lineSplit[1])
        y = float(lineSplit[2])
        if y < 8:
            continue
        tumMedian.write(line)
    tum.close()
    tumMedian.close()

    medianData = pd.read_csv('tum/' + turnType + '-median-path.tum', delim_whitespace=True, 
        names=['timestamp', 'tx', 'ty', 'tz', 'qx', 'qy', 'qz', 'qw'])
    x = medianData['tx'] + 15.6
    y = medianData['ty'] - 10.0

    ax1.plot(x, y, color=my_pal[turnType], linewidth=2.0)

#ax = sns.boxplot(x='type', y=variable, data=data, linewidth=1, width=0.6, palette=my_pal)

plt.ylim(-1.0, 7.0)
plt.yticks(ticks=[*range(0, 7)])

plt.xlim(-1.04, 15.6)
plt.xticks(ticks=[2.08 * x for x in range(0, 8)])
#plt.xticks(ticks=([2.08 * x for x in range(0, 8)] + [1]))

plt.title('Turning type paths', fontsize=20)
plt.xlabel(r'$x$ (m)', fontsize=16)
plt.ylabel(r'$y$ (m)', fontsize=16)

plt.grid(True)
plt.tight_layout()

ax2 = ax1.twiny()
ax2.set_xticks([2.08 * x for x in range(1, 8)])
ax2.set_xlim(-1.04 + 1.04, 15.6 + 1.04)
ax2.set_xticklabels([str(x) for x in range(1, 8)])
ax2.grid(b=False)

colors = {
    r'$T_{1}$\ (1-jump-t-turn)':my_pal['t1'], 
    r'$T_{2}$\ (2-jumps-t-turn)':my_pal['t2'],
    r'$\Omega_{1}$\ (1-jump-omega-turn)':my_pal['omega1'],
    r'$\Omega_{2}$\ (2-jumps-omega-turn)':my_pal['omega2'],
    r'$\Pi_{3}$\ (3-jumps-pi-turn)':my_pal['pi3'],
    r'$\Pi_{4}$\ (4-jumps-pi-turn)':my_pal['pi4'],
    r'$\Pi_{5}$\ (5-jumps-pi-turn)':my_pal['pi5'],
    r'$\Pi_{6}$\ (6-jumps-pi-turn)':my_pal['pi6']}
labels = list(colors.keys())
handles = [mlines.Line2D([], [], linewidth=2, color=colors[label]) for label in labels]
plt.legend(handles, labels)

figg = matplotlib.pyplot.gcf()
figg.set_size_inches(10, 6)

plt.tight_layout()

#plt.show()
plt.savefig('path-plot.png')

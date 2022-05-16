#!/usr/bin/env python3

#
# Generates a 2D plot of the path followed by the robot and the ideal path between swaths.
# 

import seaborn as sns
import pandas as pd
import matplotlib
matplotlib.rcParams['text.usetex'] = True
from matplotlib import pyplot as plt
import matplotlib.lines as mlines

sns.set_style("whitegrid")

paths = [
    {'pathType': 'optimal', 'title': 'Optimal path'},
    {'pathType': 'three-jumps', 'title': '3 jumps pattern'},
    {'pathType': 'trivial', 'title': 'Trivial path with reverse'},
    {'pathType': 'trivial-omega', 'title': r'Trivial path with $\Omega$-turns'}]

pathIndex = 2
pathType = paths[pathIndex]['pathType']
title = paths[pathIndex]['title']

data = pd.read_csv('tum/' + pathType + '-reference-path.tum', delim_whitespace=True,
    names=['timestamp', 'tx', 'ty', 'tz', 'qx', 'qy', 'qz', 'qw'])
#x = data['tx'] + 15.6
#y = data['ty'] + 8.0

xTrack = []
yTrack = []
xTracks = [xTrack]
yTracks = [yTrack]

lastX = data['tx'][0]
track = 0
for index, row in data.iterrows():
    if abs(lastX - row['tx']) > 1.0:
        track += 1
        xTrack = []
        xTracks.append(xTrack)
        yTrack = []
        yTracks.append(yTrack)

    xTrack.append(row['tx'])
    yTrack.append(row['ty'])
    lastX = row['tx']
#print(len(xTracks))
#print(track)

#fig, ax1 = plt.subplots(figsize=(8, 5))
fig, ax1 = plt.subplots()

for i in range(0, track + 1):
    ax1.plot([x + 15.6 for x in xTracks[i]], [y + 10.0 for y in yTracks[i]], color='#c1392b', linewidth=2.0, linestyle='--')

data = pd.read_csv('tum/' + pathType + '-full-path.tum', delim_whitespace=True,
    names=['timestamp', 'tx', 'ty', 'tz', 'qx', 'qy', 'qz', 'qw'])
x = data['tx'] + 15.6
y = data['ty'] + 10.0
ax1.plot(x, y, color='#2a80b9', linewidth=1.0)

#plt.ylim(-5, 25)
plt.ylim(-7, 27)
plt.yticks(ticks=[*range(-4, 28, 4)])

#plt.xlim(-0.4, 31.6)
plt.xlim(-1.2, 32.4)
plt.xticks(ticks=[2.08 * x for x in range(0, 16)], fontsize=8)
#plt.xticks(ticks=[0, 4])

#ax = sns.boxplot(x='type', y=variable, data=data, linewidth=1, width=0.6, palette=my_pal)

plt.title(title + ' for 15 tracks', fontsize=22, pad=20.0)
plt.xlabel(r'$x$ (m)', fontsize=14)
plt.ylabel(r'$y$ (m)', fontsize=16)

plt.grid(True)
plt.tight_layout()

ax2 = ax1.twiny()
ax2.set_xticks([2.08 * x for x in range(1, 16)])
#ax2.set_xlim(-0.4 + 1.04, 31.6 + 1.04)
ax2.set_xlim(-1.2 + 1.04, 32.4 + 1.04)
ax2.set_xticklabels([str(x) for x in range(1, 16)])
ax2.grid(b=False)
#for label in ax2.get_xticklabels():
#    label.set_color("darkgreen")

labels = ['Reference', 'Real path']
handles = [
    mlines.Line2D([], [], linewidth=1.5, color='#c1392b', linestyle='--'),
    mlines.Line2D([], [], linewidth=1.5, color='#2a80b9')]
plt.legend(handles, labels)

figg = matplotlib.pyplot.gcf()
#figg.set_size_inches(7.8 , 8)
figg.set_size_inches(7.8, 8.4)

plt.tight_layout()

#plt.show()
plt.savefig(pathType + '-path-plot.png')
